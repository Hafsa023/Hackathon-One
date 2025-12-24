"""
Configuration settings for the RAG Chatbot backend.
Loads environment variables and provides typed config access.
"""

from pydantic_settings import BaseSettings
from functools import lru_cache
from typing import Optional


class Settings(BaseSettings):
    """Application settings loaded from environment variables."""

    # TryBons/Bonsai Configuration (for LLM chat with Claude)
    trybons_api_key: str
    trybons_base_url: str = "https://go.trybons.ai/v1"
    trybons_chat_model: str = "claude-sonnet-4-20250514"

    # Cohere Configuration (for embeddings)
    cohere_api_key: str
    cohere_embedding_model: str = "embed-english-v3.0"

    # Qdrant Configuration
    qdrant_url: str
    qdrant_api_key: str
    qdrant_collection_name: str = "book_chatbot"

    # Neon Postgres Configuration (Optional - chatbot works without it)
    database_url: Optional[str] = None

    # JWT Authentication Settings
    jwt_secret_key: str = "change-this-secret-key-in-production"
    jwt_algorithm: str = "HS256"
    jwt_access_token_expire_minutes: int = 1440  # 24 hours

    # OAuth Provider Settings (Optional)
    google_client_id: Optional[str] = None
    google_client_secret: Optional[str] = None
    github_client_id: Optional[str] = None
    github_client_secret: Optional[str] = None
    linkedin_client_id: Optional[str] = None
    linkedin_client_secret: Optional[str] = None
    twitter_client_id: Optional[str] = None
    twitter_client_secret: Optional[str] = None
    frontend_url: str = "http://localhost:3000"

    # Application Settings
    chunk_size: int = 400  # Target words per chunk
    chunk_overlap: int = 50  # Overlap words between chunks
    top_k_results: int = 5  # Number of chunks to retrieve
    max_context_length: int = 4000  # Max tokens for context

    # CORS Settings
    cors_origins: str = "*"

    class Config:
        env_file = ".env"
        env_file_encoding = "utf-8"


@lru_cache()
def get_settings() -> Settings:
    """Get cached settings instance."""
    return Settings()
