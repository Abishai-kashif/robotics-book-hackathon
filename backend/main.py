"""
Main application entry point for the chatbot backend
"""
import uvicorn
import logging
from fastapi import FastAPI
from fastapi.middleware.cors import CORSMiddleware

from src.api.chatbot_router import router as chatbot_router
from src.utils.metrics import time_it

# Set up logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

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