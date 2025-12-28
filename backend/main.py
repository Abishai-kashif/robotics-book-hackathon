"""
Main application entry point for the chatbot backend
"""
import uvicorn
import logging
import sys
from fastapi import FastAPI
from fastapi.middleware.cors import CORSMiddleware

from src.api.chatbot_router import router as chatbot_router
from src.utils.metrics import time_it
from src.config.env import get_environment_config

# Set up logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

# Load and validate environment configuration at startup
try:
    config = get_environment_config()
    logger.info("Environment configuration loaded successfully")
    logger.info(f"LLM Provider: {config.get_llm_provider()}")
    logger.info(f"Model: {config.model_name}")
    logger.info(f"Qdrant Cloud: {config.qdrant_cluster_endpoint}")
    logger.info(f"Embedding Model: {config.embedding_model}")
except ValueError as e:
    logger.error(f"Environment configuration validation failed: {e}")
    logger.error("Please check your .env file or environment variables")
    sys.exit(1)

app = FastAPI(
    title="Docusaurus Chatbot API",
    description="API for conversational chatbot integration with textbook content using RAG",
    version="1.0.0"
)

# Add CORS middleware
app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],  # In production, replace with specific origins
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

# Include API routes
app.include_router(chatbot_router, prefix="/api/v1")

@app.get("/")
async def root():
    """
    Root endpoint for the API
    """
    return {"message": "Docusaurus Chatbot API", "status": "running"}

if __name__ == "__main__":
    uvicorn.run(
        "main:app",
        host="0.0.0.0",
        port=8000,
        reload=True  # Enable auto-reload during development
    )