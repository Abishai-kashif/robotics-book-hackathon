"""
Embedding model for the chatbot application
"""
from pydantic import BaseModel, Field
from typing import List, Optional, Dict, Any
from datetime import datetime
from uuid import uuid4

class VectorEmbedding(BaseModel):
    """
    Numerical representation of textbook content for semantic search
    """
    embedding_id: str = Field(default_factory=lambda: f"emb_{uuid4().hex}")
    content_id: str
    vector: List[float]
    model_used: str
    created_at: datetime = Field(default_factory=datetime.utcnow)

class TextbookContent(BaseModel):
    """
    Represents a chunk of textbook content that has been processed for vector search
    """
    content_id: str = Field(default_factory=lambda: f"content_{uuid4().hex}")
    title: str
    body: str
    source_path: str
    embedding: List[float]
    metadata: Optional[Dict[str, Any]] = None

class ConversationSession(BaseModel):
    """
    Maintains context for multi-turn conversations
    """
    session_id: str = Field(default_factory=lambda: f"sess_{uuid4().hex}")
    user_id: Optional[str] = None
    created_at: datetime = Field(default_factory=datetime.utcnow)
    last_activity: datetime = Field(default_factory=datetime.utcnow)
    history: List[Dict[str, Any]] = Field(default_factory=list)