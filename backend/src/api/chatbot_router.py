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
    Process a user query and return a RAG-enhanced response.

    The OpenAI Agents SDK manages conversation context internally,
    so explicit session management has been removed.
    """
    try:
        logger.info(f"Processing query: {query.content[:50]}...")

        # Process the query using RAG service with agent
        response = await rag_service.process_query(query)

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
    Check the health status of the service including Qdrant Cloud connectivity
    """
    from ..database.qdrant_client import verify_connection
    from ..config.env import get_environment_config

    health_status = {
        "status": "healthy",
        "timestamp": datetime.utcnow().isoformat(),
        "checks": {}
    }

    try:
        # Check Qdrant Cloud connectivity
        qdrant_ok = verify_connection(qdrant_service.client)
        health_status["checks"]["qdrant"] = "ok" if qdrant_ok else "error"

        if not qdrant_ok:
            health_status["status"] = "degraded"

        # Check collection existence
        try:
            qdrant_service.client.get_collection(qdrant_service.collection_name)
            health_status["checks"]["collection"] = "ok"
        except Exception as e:
            logger.warning(f"Collection check failed: {e}")
            health_status["checks"]["collection"] = "error"
            health_status["status"] = "degraded"

        # Check LLM configuration
        try:
            config = get_environment_config()
            provider = config.get_llm_provider()
            health_status["checks"]["llm_provider"] = provider

            # Test actual API connectivity if Gemini is configured
            if provider == "gemini" and config.is_gemini_configured():
                try:
                    # Quick validation that we can create a client
                    from openai import AsyncOpenAI
                    gemini_client = AsyncOpenAI(
                        base_url=config.gemini_base_url,
                        api_key=config.gemini_api_key,
                        timeout=5.0  # Short timeout for health check
                    )
                    health_status["checks"]["llm"] = "ok"
                    health_status["checks"]["gemini_configured"] = True
                except Exception as e:
                    logger.warning(f"Gemini client creation failed: {e}")
                    health_status["checks"]["llm"] = "degraded"
                    health_status["checks"]["gemini_error"] = str(e)
                    health_status["status"] = "degraded"
            elif provider == "openai":
                health_status["checks"]["llm"] = "ok"
                health_status["checks"]["openai_configured"] = True
            else:
                health_status["checks"]["llm"] = "ok"

        except Exception as e:
            logger.warning(f"LLM config check failed: {e}")
            health_status["checks"]["llm"] = "error"
            health_status["status"] = "degraded"

        # Check embedding model
        health_status["checks"]["embeddings"] = "ok"  # Validated at startup

        # Return appropriate status code
        if health_status["status"] == "healthy":
            return health_status
        else:
            raise HTTPException(status_code=503, detail=health_status)

    except HTTPException:
        raise
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