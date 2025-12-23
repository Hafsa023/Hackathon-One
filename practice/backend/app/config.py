"""
Configuration settings for the RAG Chatbot backend.
Loads environment variables and provides typed config access.
"""

from pydantic_settings import BaseSettings
from functools import lru_cache
from typing import Optional


class Settings(BaseSettings):
    """Application settings loaded from environment variables."""

    # Qwen/OpenRouter Configuration
    qwen_api_key: str
    qwen_base_url: str = "https://openrouter.ai/api/v1"
    qwen_embedding_model: str = "qwen/qwen-turbo"  # For embeddings fallback
    qwen_chat_model: str = "qwen/qwen-2.5-72b-instruct"  # Main chat model

    # Google API Configuration (for embeddings)
    google_api_key: Optional[str] = None

    # Qdrant Configuration
    qdrant_url: str
    qdrant_api_key: str
    qdrant_collection_name: str = "book_chatbot"

    # Neon Postgres Configuration
    database_url: str

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
