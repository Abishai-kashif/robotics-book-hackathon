"""
RAG (Retrieval-Augmented Generation) service for processing queries and generating responses
"""
import logging
from typing import List, Optional, Dict, Any
from sentence_transformers import SentenceTransformer
import os
from datetime import datetime
from uuid import uuid4

from qdrant_client.http import models

from ..models.query import UserQuery
from ..models.response import RAGResponse, SourceCitation
from ..models.embedding import TextbookContent
from .qdrant_service import QdrantService
from ..agents.chatbot import get_global_agent, process_query_with_agent
from ..config.env import get_environment_config

logger = logging.getLogger(__name__)

class RAGService:
    def __init__(self, qdrant_service: QdrantService):
        self.qdrant_service = qdrant_service
        self.config = get_environment_config()
        self._encoder = None  # Initialize encoder lazily
        self.sessions = {}  # In-memory session storage (use Redis in production)

    @property
    def encoder(self):
        """Lazily initialize the sentence transformer encoder to avoid download at startup"""
        if self._encoder is None:
            from sentence_transformers import SentenceTransformer
            logger.info(f"Loading embedding model: {self.config.embedding_model}")
            self._encoder = SentenceTransformer(self.config.embedding_model)
            logger.info("Embedding model loaded successfully")
        return self._encoder

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

            # Combine context for the agent
            context = "\n\n".join(context_texts)

            # Generate response using agent (replaces direct LLM calls)
            agent_result = await process_query_with_agent(
                query=user_query.content,
                context=context,
                agent=get_global_agent()
            )

            # Check if agent processing succeeded
            if not agent_result["success"]:
                logger.error(f"Agent processing failed: {agent_result['error']}")
                return RAGResponse(
                    query_id=user_query.query_id,
                    answer=f"I encountered an error processing your query: {agent_result['error']}",
                    sources=sources,
                    confidence_score=0.0
                )

            answer = agent_result["answer"]

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

    # Session management methods removed - OpenAI Agents SDK manages sessions internally
    # The Agent SDK maintains conversation context automatically through the Runner