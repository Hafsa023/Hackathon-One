"""
Embedding service using Cohere API.
Uses Cohere's embed-english-v3.0 model for high-quality embeddings.
"""

import cohere
from typing import List
import time

from .config import get_settings

settings = get_settings()


class EmbeddingService:
    """Service for generating text embeddings using Cohere."""

    def __init__(self):
        self.client = cohere.ClientV2(settings.cohere_api_key)
        self.model = settings.cohere_embedding_model
        self.embedding_dim = 1024  # embed-english-v3.0 outputs 1024 dimensions
        print(f"Embedding service initialized (Cohere {self.model}, dim={self.embedding_dim})")

    def get_embedding(self, text: str) -> List[float]:
        """Get embedding vector for a single text (for documents)."""
        # Truncate long texts
        text = text[:8000] if len(text) > 8000 else text

        response = self.client.embed(
            texts=[text],
            model=self.model,
            input_type="search_document",
            embedding_types=["float"],
        )
        return response.embeddings.float_[0]

    def get_query_embedding(self, text: str) -> List[float]:
        """Get embedding for search query (uses search_query input type)."""
        # Truncate long texts
        text = text[:8000] if len(text) > 8000 else text

        response = self.client.embed(
            texts=[text],
            model=self.model,
            input_type="search_query",
            embedding_types=["float"],
        )
        return response.embeddings.float_[0]

    def get_embeddings_batch(self, texts: List[str], batch_size: int = 96) -> List[List[float]]:
        """Get embeddings for multiple texts with batching (for documents)."""
        all_embeddings = []

        for i in range(0, len(texts), batch_size):
            batch = texts[i:i + batch_size]
            # Truncate each text
            batch = [t[:8000] if len(t) > 8000 else t for t in batch]

            print(f"    Getting embeddings for batch {i // batch_size + 1}/{(len(texts) + batch_size - 1) // batch_size}...")

            response = self.client.embed(
                texts=batch,
                model=self.model,
                input_type="search_document",
                embedding_types=["float"],
            )

            # Extract embeddings
            batch_embeddings = response.embeddings.float_
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
