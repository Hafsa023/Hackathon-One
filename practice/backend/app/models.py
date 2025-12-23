"""
Pydantic models for request/response schemas.
"""

from pydantic import BaseModel, Field
from typing import Optional, List
from datetime import datetime


# Request Models
class ChatRequest(BaseModel):
    """Request model for chat endpoint."""
    question: str = Field(..., min_length=1, max_length=2000, description="User's question")
    selected_text: Optional[str] = Field(None, max_length=10000, description="User-highlighted text from the book")
    session_id: Optional[str] = Field(None, description="Session ID for chat history tracking")
    chapter: Optional[str] = Field(None, description="Current chapter/page context")


class IngestRequest(BaseModel):
    """Request model for manual ingestion trigger."""
    force_reindex: bool = Field(False, description="Force re-indexing even if content exists")


# Response Models
class Source(BaseModel):
    """Source reference for an answer."""
    chapter: str
    section: Optional[str] = None
    relevance_score: float


class ChatResponse(BaseModel):
    """Response model for chat endpoint."""
    answer: str
    sources: List[Source]
    session_id: str
    timestamp: datetime = Field(default_factory=datetime.utcnow)


class HealthResponse(BaseModel):
    """Health check response."""
    status: str
    qdrant_connected: bool
    database_connected: bool
    version: str = "1.0.0"


# Database Models (for SQLAlchemy)
class ChatHistoryCreate(BaseModel):
    """Model for creating chat history records."""
    session_id: str
    question: str
    answer: str
    chapter: Optional[str] = None
    sources: List[str] = []


class ChatHistoryResponse(BaseModel):
    """Response model for chat history."""
    id: int
    session_id: str
    question: str
    answer: str
    chapter: Optional[str]
    created_at: datetime
