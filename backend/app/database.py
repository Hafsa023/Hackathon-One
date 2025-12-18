"""
Database connection and models for Neon Serverless Postgres.
Handles chat history storage and session tracking.
Database is OPTIONAL - chatbot works without it.
"""

from sqlalchemy import create_engine, Column, Integer, String, Text, DateTime, JSON
from sqlalchemy.ext.declarative import declarative_base
from sqlalchemy.orm import sessionmaker
from datetime import datetime
from contextlib import contextmanager
from typing import Optional

from .config import get_settings

settings = get_settings()

# Database is optional - check if valid connection string provided
_db_enabled = False
engine = None
SessionLocal = None
Base = declarative_base()

try:
    if (settings.database_url
        and settings.database_url.startswith("postgresql://")
        and "YOUR_PASSWORD" not in settings.database_url):
        engine = create_engine(
            settings.database_url,
            pool_pre_ping=True,
            pool_size=5,
            max_overflow=10,
            echo=False,
        )
        SessionLocal = sessionmaker(autocommit=False, autoflush=False, bind=engine)
        _db_enabled = True
        print("Database connection configured")
    else:
        print("Database not configured - chat history disabled")
except Exception as e:
    print(f"Database connection failed: {e} - chat history disabled")


def is_db_enabled() -> bool:
    """Check if database is available."""
    return _db_enabled


class User(Base):
    """User table for authentication."""
    __tablename__ = "users"

    id = Column(Integer, primary_key=True, index=True)
    name = Column(String(255), nullable=False)
    email = Column(String(255), unique=True, index=True, nullable=False)
    hashed_password = Column(String(255), nullable=True)  # Nullable for OAuth users
    oauth_provider = Column(String(50), nullable=True)  # google, github, linkedin, twitter
    oauth_id = Column(String(255), nullable=True)  # Provider's user ID
    avatar_url = Column(String(500), nullable=True)  # Profile picture URL
    created_at = Column(DateTime, default=datetime.utcnow, index=True)


class ChatHistory(Base):
    """Chat history table for storing conversations."""
    __tablename__ = "chat_history"

    id = Column(Integer, primary_key=True, index=True)
    session_id = Column(String(64), index=True, nullable=False)
    question = Column(Text, nullable=False)
    answer = Column(Text, nullable=False)
    chapter = Column(String(255), nullable=True)
    sources = Column(JSON, default=[])
    created_at = Column(DateTime, default=datetime.utcnow, index=True)


class AnalyticsEvent(Base):
    """Analytics events for tracking usage patterns."""
    __tablename__ = "analytics_events"

    id = Column(Integer, primary_key=True, index=True)
    event_type = Column(String(50), nullable=False, index=True)
    session_id = Column(String(64), index=True)
    chapter = Column(String(255), nullable=True)
    event_data = Column(JSON, default={})  # renamed from metadata (reserved)
    created_at = Column(DateTime, default=datetime.utcnow, index=True)


def init_db():
    """Initialize database tables."""
    if _db_enabled and engine:
        Base.metadata.create_all(bind=engine)
        print("Database tables initialized")
    else:
        print("Skipping database initialization (not configured)")


@contextmanager
def get_db():
    """Get database session context manager."""
    if not _db_enabled or not SessionLocal:
        yield None
        return

    db = SessionLocal()
    try:
        yield db
        db.commit()
    except Exception:
        db.rollback()
        raise
    finally:
        db.close()


def get_db_session():
    """Dependency for FastAPI routes."""
    if not _db_enabled or not SessionLocal:
        yield None
        return

    db = SessionLocal()
    try:
        yield db
    finally:
        db.close()


# Repository functions
def save_chat_history(
    db,
    session_id: str,
    question: str,
    answer: str,
    chapter: str = None,
    sources: list = None
) -> Optional[ChatHistory]:
    """Save a chat exchange to history."""
    if db is None:
        return None

    record = ChatHistory(
        session_id=session_id,
        question=question,
        answer=answer,
        chapter=chapter,
        sources=sources or [],
    )
    db.add(record)
    db.commit()
    db.refresh(record)
    return record


def get_session_history(db, session_id: str, limit: int = 10) -> list:
    """Retrieve chat history for a session."""
    if db is None:
        return []

    return (
        db.query(ChatHistory)
        .filter(ChatHistory.session_id == session_id)
        .order_by(ChatHistory.created_at.desc())
        .limit(limit)
        .all()
    )


def log_analytics_event(
    db,
    event_type: str,
    session_id: str = None,
    chapter: str = None,
    event_data: dict = None
):
    """Log an analytics event."""
    if db is None:
        return

    event = AnalyticsEvent(
        event_type=event_type,
        session_id=session_id,
        chapter=chapter,
        event_data=event_data or {},
    )
    db.add(event)
    db.commit()
