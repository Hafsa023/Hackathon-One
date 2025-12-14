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
