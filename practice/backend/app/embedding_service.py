"""
Embedding service using OpenRouter API with Qwen.
Uses your existing OpenRouter API key for embeddings.
"""

from openai import OpenAI
from typing import List
import time

from .config import get_settings

settings = get_settings()


class EmbeddingService:
    """Service for generating text embeddings using DashScope."""

    def __init__(self):
        self.client = OpenAI(
            api_key=settings.qwen_api_key,
            base_url=settings.qwen_base_url,
        )
        # Use DashScope's embedding model
        self.model = settings.qwen_embedding_model
        self.embedding_dim = 1024  # text-embedding-v3 outputs 1024 dimensions
        print(f"Embedding service initialized (DashScope {self.model}, dim={self.embedding_dim})")

    def get_embedding(self, text: str) -> List[float]:
        """Get embedding vector for a single text."""
        # Truncate long texts
        text = text[:8000] if len(text) > 8000 else text

        response = self.client.embeddings.create(
            model=self.model,
            input=text,
        )
        return response.data[0].embedding

    def get_query_embedding(self, text: str) -> List[float]:
        """Get embedding for search query (same as document embedding for this model)."""
        return self.get_embedding(text)

    def get_embeddings_batch(self, texts: List[str], batch_size: int = 50) -> List[List[float]]:
        """Get embeddings for multiple texts with batching."""
        all_embeddings = []

        for i in range(0, len(texts), batch_size):
            batch = texts[i:i + batch_size]
            # Truncate each text
            batch = [t[:8000] if len(t) > 8000 else t for t in batch]

            print(f"    Getting embeddings for batch {i // batch_size + 1}/{(len(texts) + batch_size - 1) // batch_size}...")

            response = self.client.embeddings.create(
                model=self.model,
                input=batch,
            )

            # Extract embeddings in order
            batch_embeddings = [item.embedding for item in response.data]
            all_embeddings.extend(batch_embeddings)

            # Small delay to avoid rate limits
            if i + batch_size < len(texts):
                time.sleep(0.5)

        return all_embeddings


# Singleton instance
_embedding_service = None


def get_embedding_service() -> EmbeddingService:
    """Get or create embedding service singleton."""
    global _embedding_service
    if _embedding_service is None:
        _embedding_service = EmbeddingService()
    return _embedding_service
