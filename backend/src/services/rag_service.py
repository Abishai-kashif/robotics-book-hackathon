"""
RAG (Retrieval-Augmented Generation) service for processing queries and generating responses
"""
import logging
from typing import List, Optional, Dict, Any
from sentence_transformers import SentenceTransformer
import openai
import os
from datetime import datetime
from uuid import uuid4

from qdrant_client.http import models

from ..models.query import UserQuery
from ..models.response import RAGResponse, SourceCitation
from ..models.embedding import TextbookContent, ConversationSession
from .qdrant_service import QdrantService

logger = logging.getLogger(__name__)

class RAGService:
    def __init__(self, qdrant_service: QdrantService):
        self.qdrant_service = qdrant_service
        self.embedding_model = os.getenv("EMBEDDING_MODEL", "all-MiniLM-L6-v2")
        self._encoder = None  # Initialize encoder lazily
        self._gemini_model = None  # Initialize Gemini lazily
        self.openai_api_key = os.getenv("OPENAI_API_KEY")
        self.gemini_api_key = os.getenv("GEMINI_API_KEY")
        if self.openai_api_key:
            openai.api_key = self.openai_api_key
        self.sessions = {}  # In-memory session storage (use Redis in production)

    @property
    def encoder(self):
        """Lazily initialize the sentence transformer encoder to avoid download at startup"""
        if self._encoder is None:
            from sentence_transformers import SentenceTransformer
            logger.info(f"Loading embedding model: {self.embedding_model}")
            self._encoder = SentenceTransformer(self.embedding_model)
            logger.info("Embedding model loaded successfully")
        return self._encoder

    @property
    def gemini_model(self):
        """Lazily initialize the Gemini model"""
        if self._gemini_model is None and self.gemini_api_key:
            try:
                from google import genai
                client = genai.Client(api_key=self.gemini_api_key)
                self._gemini_model = client
                logger.info("Gemini model initialized successfully with google.genai")
            except ImportError:
                # Fallback to old package if new one not installed
                import google.generativeai as genai
                genai.configure(api_key=self.gemini_api_key)
                # Use gemini-flash-latest which is an available stable model
                self._gemini_model = genai.GenerativeModel('gemini-flash-latest')
                logger.info("Gemini model initialized successfully with google.generativeai (deprecated)")
        return self._gemini_model

    async def process_query(self, user_query: UserQuery) -> RAGResponse:
        """
        Process a user query and return a RAG-enhanced response
        """
        try:
            logger.info(f"Processing query: {user_query.content}")

            # Generate embedding for the query
            query_embedding = self.encoder.encode(user_query.content).tolist()

            # Search for similar content in the vector database
            # If there's context (current page), we'll try to prioritize content from that page
            similar_contents = []
            if user_query.source_page:
                # First, try to find content specifically from the current page
                page_specific_contents = self._search_by_source_path(user_query.source_page, limit=2)

                # Then get general similar content
                general_contents = self.qdrant_service.search_similar(query_embedding, limit=5)

                # Combine results, prioritizing page-specific content
                # Add page-specific results first (with potential boost)
                for item in page_specific_contents:
                    item["score"] *= 1.2  # Boost score for page-specific content
                    if item not in similar_contents:
                        similar_contents.append(item)

                # Add general results, avoiding duplicates
                for item in general_contents:
                    if item not in similar_contents:
                        similar_contents.append(item)

                # Keep only top 5 results after combining
                similar_contents = sorted(similar_contents, key=lambda x: x["score"], reverse=True)[:5]
            else:
                # Standard search without context
                similar_contents = self.qdrant_service.search_similar(query_embedding, limit=5)

            if not similar_contents:
                # Return a response when no relevant content is found
                return RAGResponse(
                    query_id=user_query.query_id,
                    answer="I couldn't find relevant content in the textbook to answer your question. Please try rephrasing or ask about a different topic.",
                    sources=[],
                    confidence_score=0.0
                )

            # Prepare context from retrieved content
            context_texts = []
            sources = []
            for item in similar_contents:
                content = item["payload"]
                context_texts.append(content.get("body", ""))

                source = SourceCitation(
                    response_id=f"resp_{uuid4().hex[:8]}",
                    content_id=item["content_id"],
                    text_snippet=content.get("body", "")[:200] + "..." if len(content.get("body", "")) > 200 else content.get("body", ""),
                    source_path=content.get("source_path", ""),
                    page_reference=content.get("metadata", {}).get("chapter", "")
                )
                sources.append(source)

            # Combine context for the LLM
            context = "\n\n".join(context_texts)

            # Generate response using LLM
            answer = await self._generate_answer(user_query.content, context)

            # Calculate a basic confidence score based on similarity scores
            avg_similarity = sum(item["score"] for item in similar_contents) / len(similar_contents)
            confidence_score = min(avg_similarity, 1.0)  # Ensure it's between 0 and 1

            # Create and return the response
            response = RAGResponse(
                query_id=user_query.query_id,
                answer=answer,
                sources=sources,
                confidence_score=confidence_score
            )

            return response
        except Exception as e:
            logger.error(f"Error processing query: {e}")
            raise

    def _search_by_source_path(self, source_path: str, limit: int = 5):
        """
        Search for content specifically from a given source path
        """
        try:
            # Use Qdrant's payload filtering to find content from a specific source
            results = self.qdrant_service.client.scroll(
                collection_name=self.qdrant_service.collection_name,
                scroll_filter=models.Filter(
                    must=[
                        models.FieldCondition(
                            key="source_path",
                            match=models.MatchValue(value=source_path)
                        )
                    ]
                ),
                limit=limit
            )

            # Convert results to the expected format
            formatted_results = []
            for record in results[0]:  # results is (records, next_page_offset)
                formatted_results.append({
                    "content_id": record.id,
                    "payload": record.payload,
                    "score": 1.0  # Default score for page-specific content
                })

            return formatted_results
        except Exception as e:
            logger.error(f"Error searching by source path: {e}")
            return []

    async def _generate_answer(self, query: str, context: str) -> str:
        """
        Generate an answer using the LLM based on the query and context
        """
        system_prompt = """You are a helpful assistant for a Physical AI & Humanoid Robotics textbook.
Answer questions based on the provided context from the textbook.
Be accurate, informative, and cite relevant information from the context.
If the context doesn't contain enough information to fully answer the question, say so."""

        user_prompt = f"""Context from the textbook:
{context}

Question: {query}

Please provide a helpful and accurate answer based on the context above."""

        try:
            # Try Gemini first (free tier available)
            if self.gemini_api_key and self.gemini_model:
                logger.info("Using Gemini for response generation")

                # Check if using new google.genai API
                if hasattr(self.gemini_model, 'models'):
                    # New google.genai API
                    response = self.gemini_model.models.generate_content(
                        model='gemini-flash-latest',
                        contents=f"{system_prompt}\n\n{user_prompt}",
                        config={
                            "temperature": 0.7,
                            "max_output_tokens": 1000,
                        }
                    )
                    return response.text.strip()
                else:
                    # Old google.generativeai API
                    response = self.gemini_model.generate_content(
                        f"{system_prompt}\n\n{user_prompt}",
                        generation_config={
                            "temperature": 0.7,
                            "max_output_tokens": 1000,
                        }
                    )
                    return response.text.strip()

            # Fallback to OpenAI if available
            elif self.openai_api_key:
                logger.info("Using OpenAI for response generation")
                response = openai.ChatCompletion.create(
                    model="gpt-3.5-turbo",
                    messages=[
                        {"role": "system", "content": system_prompt},
                        {"role": "user", "content": user_prompt}
                    ],
                    max_tokens=500,
                    temperature=0.7
                )
                return response.choices[0].message.content.strip()

            else:
                # No API key configured
                logger.warning("No LLM API key configured")
                return f"Based on the textbook content, here's an answer to your question: '{query}'. [Note: No LLM API key configured. Please set GEMINI_API_KEY or OPENAI_API_KEY in your .env file]"

        except Exception as e:
            logger.error(f"Error generating answer with LLM: {e}")
            return f"I found relevant content but encountered an error generating a response. Error: {str(e)}"

    async def store_content_embeddings(self, content: TextbookContent):
        """
        Process and store embeddings for textbook content
        """
        try:
            # Generate embedding for the content
            embedding = self.encoder.encode(content.body).tolist()

            # Store in Qdrant
            payload = {
                "title": content.title,
                "body": content.body,
                "source_path": content.source_path,
                "metadata": content.metadata or {}
            }

            self.qdrant_service.store_embeddings(
                content_id=content.content_id,
                embedding=embedding,
                payload=payload
            )

            logger.info(f"Stored embeddings for content: {content.title}")
        except Exception as e:
            logger.error(f"Error storing content embeddings: {e}")
            raise

    async def create_conversation_session(self, user_id: Optional[str] = None) -> ConversationSession:
        """
        Create a new conversation session
        """
        session = ConversationSession(user_id=user_id)
        self.sessions[session.session_id] = session
        return session

    async def get_conversation_session(self, session_id: str) -> Optional[ConversationSession]:
        """
        Get an existing conversation session
        """
        return self.sessions.get(session_id)

    async def add_message_to_session(self, session_id: str, message: Dict[str, Any]):
        """
        Add a message to a conversation session
        """
        session = self.sessions.get(session_id)
        if session:
            session.history.append(message)
            # Update last activity time
            session.last_activity = datetime.utcnow()
        else:
            # Create a new session if it doesn't exist
            session = await self.create_conversation_session()
            session.history.append(message)
            return session.session_id
        return session_id

    async def get_conversation_history(self, session_id: str) -> List[Dict[str, Any]]:
        """
        Get the conversation history for a session
        """
        session = self.sessions.get(session_id)
        if session:
            return session.history
        return []