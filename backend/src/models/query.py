"""
Query model for the chatbot application
"""
from pydantic import BaseModel, Field
from typing import Optional, Dict, Any
from datetime import datetime
from uuid import uuid4

class UserQuery(BaseModel):
    """
    Represents a natural language query submitted by a student
    """
    query_id: str = Field(default_factory=lambda: f"query_{uuid4().hex}")
    timestamp: datetime = Field(default_factory=datetime.utcnow)
    user_context: Optional[Dict[str, Any]] = None
    source_page: Optional[str] = None
    session_id: Optional[str] = None
    content: str = Field(..., min_length=2, max_length=1000)