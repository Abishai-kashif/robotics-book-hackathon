"""
Response model for the chatbot application
"""
from pydantic import BaseModel, Field
from typing import List, Optional
from datetime import datetime
from uuid import uuid4

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