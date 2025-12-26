"""
Data models for the chatbot application
"""
from pydantic import BaseModel, Field
from typing import List, Optional, Dict, Any
from datetime import datetime
from uuid import uuid4

class UserQuery(BaseModel):
    """
    Represents a natural language query submitted by a student
    """
    query_id: str = Field(default_factory=lambda: f"query_{uuid4().hex}")
    content: str = Field(..., min_length=5, max_length=1000)
    timestamp: datetime = Field(default_factory=datetime.utcnow)
    user_context: Optional[Dict[str, Any]] = None
    source_page: Optional[str] = None

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

class VectorEmbedding(BaseModel):
    """
    Numerical representation of textbook content for semantic search
    """
    embedding_id: str = Field(default_factory=lambda: f"emb_{uuid4().hex}")
    content_id: str
    vector: List[float]
    model_used: str
    created_at: datetime = Field(default_factory=datetime.utcnow)

class SourceCitation(BaseModel):
    """
    Reference to specific parts of the textbook content that support the response
    """
    citation_id: str = Field(default_factory=lambda: f"cite_{uuid4().hex}")
    response_id: str
    content_id: str
    text_snippet: str
    source_path: str
    page_reference: Optional[str] = None

class RAGResponse(BaseModel):
    """
    The generated response that combines retrieved content with conversational AI
    """
    response_id: str = Field(default_factory=lambda: f"resp_{uuid4().hex}")
    query_id: str
    answer: str
    sources: List[SourceCitation]
    confidence_score: float = Field(ge=0.0, le=1.0)
    timestamp: datetime = Field(default_factory=datetime.utcnow)

class ConversationSession(BaseModel):
    """
    Maintains context for multi-turn conversations
    """
    session_id: str = Field(default_factory=lambda: f"sess_{uuid4().hex}")
    user_id: Optional[str] = None
    created_at: datetime = Field(default_factory=datetime.utcnow)
    last_activity: datetime = Field(default_factory=datetime.utcnow)
    history: List[Dict[str, Any]] = Field(default_factory=list)