"""
API router for chatbot functionality
"""
from fastapi import APIRouter, HTTPException, BackgroundTasks
from typing import List, Optional
import logging
from datetime import datetime

from ..models.query import UserQuery
from ..models.response import RAGResponse
from ..models.embedding import TextbookContent
from ..services.qdrant_service import QdrantService
from ..services.rag_service import RAGService

logger = logging.getLogger(__name__)
router = APIRouter()

# Initialize services
qdrant_service = QdrantService()
rag_service = RAGService(qdrant_service)

@router.post("/chat", response_model=RAGResponse)
async def chat_endpoint(query: UserQuery):
    """
    Process a user query and return a RAG-enhanced response
    """
    try:
        logger.info(f"Processing query: {query.content[:50]}...")

        # Process the query using RAG service
        response = await rag_service.process_query(query)

        # Add messages to conversation session if session ID is provided
        if query.session_id:
            # Add user message to session
            await rag_service.add_message_to_session(
                query.session_id,
                {"role": "user", "content": query.content, "timestamp": query.timestamp}
            )

            # Add bot response to session
            await rag_service.add_message_to_session(
                query.session_id,
                {
                    "role": "assistant",
                    "content": response.answer,
                    "sources": response.sources,
                    "timestamp": response.timestamp
                }
            )
        else:
            # Create a new session if one wasn't provided
            session = await rag_service.create_conversation_session()
            query.session_id = session.session_id

            # Add messages to the new session
            await rag_service.add_message_to_session(
                session.session_id,
                {"role": "user", "content": query.content, "timestamp": query.timestamp}
            )

            await rag_service.add_message_to_session(
                session.session_id,
                {
                    "role": "assistant",
                    "content": response.answer,
                    "sources": response.sources,
                    "timestamp": response.timestamp
                }
            )

        logger.info(f"Generated response with ID: {response.response_id}")
        return response
    except Exception as e:
        logger.error(f"Error processing query: {e}")
        raise HTTPException(status_code=500, detail=f"Error processing query: {str(e)}")

@router.post("/embeddings")
async def store_embeddings_endpoint(content: TextbookContent):
    """
    Process and store new content embeddings
    """
    try:
        logger.info(f"Processing embeddings for content: {content.title}")

        # Store embeddings using Qdrant service
        await rag_service.store_content_embeddings(content)

        logger.info(f"Successfully stored embeddings for content: {content.content_id}")
        return {"message": "Embeddings created successfully", "content_id": content.content_id}
    except Exception as e:
        logger.error(f"Error storing embeddings: {e}")
        raise HTTPException(status_code=500, detail=f"Error storing embeddings: {str(e)}")

@router.get("/health")
async def health_check():
    """
    Check the health status of the service
    """
    try:
        # Check if Qdrant is accessible
        qdrant_service.client.get_collection(qdrant_service.collection_name)

        return {
            "status": "healthy",
            "timestamp": datetime.utcnow().isoformat(),
            "services": {
                "qdrant": "connected",
                "api": "operational"
            }
        }
    except Exception as e:
        logger.error(f"Health check failed: {e}")
        raise HTTPException(status_code=503, detail="Service unavailable")

@router.get("/metrics")
async def get_metrics():
    """
    Get performance metrics for the service
    """
    try:
        from src.utils.metrics import metrics_collector
        return metrics_collector.get_metrics_summary()
    except Exception as e:
        logger.error(f"Error getting metrics: {e}")
        raise HTTPException(status_code=500, detail="Error retrieving metrics")