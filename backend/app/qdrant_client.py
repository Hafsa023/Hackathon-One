"""
Qdrant vector database client for storing and retrieving book embeddings.
"""

from qdrant_client import QdrantClient, models
from qdrant_client.http.models import Distance, VectorParams, PointStruct
from typing import List, Dict, Optional
import hashlib

from .config import get_settings
from .embedding_service import get_embedding_service

settings = get_settings()


class QdrantService:
    """Service for managing Qdrant vector operations."""

    def __init__(self):
        self.client = QdrantClient(
            url=settings.qdrant_url,
            api_key=settings.qdrant_api_key,
            timeout=120,  # Increase timeout for large uploads
        )
        self.collection_name = settings.qdrant_collection_name
        self.embedding_service = get_embedding_service()
        self.vector_size = self.embedding_service.embedding_dim

    def ensure_collection_exists(self):
        """Create collection if it doesn't exist."""
        collections = self.client.get_collections().collections
        collection_names = [c.name for c in collections]

        if self.collection_name not in collection_names:
            self.client.create_collection(
                collection_name=self.collection_name,
                vectors_config=VectorParams(
                    size=self.vector_size,
                    distance=Distance.COSINE,
                ),
            )
            print(f"Created collection: {self.collection_name}")
        else:
            print(f"Collection {self.collection_name} already exists")

    def add_texts(self, texts: List[str], metadata_list: List[Dict]) -> int:
        """
        Add texts with metadata, computing embeddings via API.
        """
        # Get all embeddings in batch for efficiency
        print(f"  Computing embeddings for {len(texts)} texts...")
        embeddings = self.embedding_service.get_embeddings_batch(texts)

        points = []
        for i, (text, meta, embedding) in enumerate(zip(texts, metadata_list, embeddings)):
            chunk_id = self._generate_id(text, meta.get("chapter", ""))

            point = PointStruct(
                id=chunk_id,
                vector=embedding,
                payload={"text": text, **meta},
            )
            points.append(point)

        # Upsert in smaller batches to avoid timeout
        batch_size = 20
        for i in range(0, len(points), batch_size):
            batch = points[i:i + batch_size]
            self.client.upsert(
                collection_name=self.collection_name,
                points=batch,
            )
            print(f"    Uploaded {min(i + batch_size, len(points))}/{len(points)} points to Qdrant")

        return len(texts)

    def upsert_chunks(self, chunks: List[Dict]) -> int:
        """
        Insert or update document chunks with pre-computed embeddings.
        """
        points = []
        for chunk in chunks:
            chunk_id = self._generate_id(chunk["text"], chunk["chapter"])

            point = PointStruct(
                id=chunk_id,
                vector=chunk["embedding"],
                payload={
                    "text": chunk["text"],
                    "chapter": chunk["chapter"],
                    "section": chunk.get("section", ""),
                    "position": chunk.get("position", 0),
                    "word_count": len(chunk["text"].split()),
                },
            )
            points.append(point)

        batch_size = 100
        for i in range(0, len(points), batch_size):
            batch = points[i:i + batch_size]
            self.client.upsert(
                collection_name=self.collection_name,
                points=batch,
            )

        return len(points)

    def search_with_text(
        self,
        query_text: str,
        top_k: int = 5,
        chapter_filter: Optional[str] = None,
    ) -> List[Dict]:
        """
        Search using text query - compute embedding then search.
        Uses query-optimized embedding if available.
        """
        # Use query embedding if service supports it
        if hasattr(self.embedding_service, 'get_query_embedding'):
            query_embedding = self.embedding_service.get_query_embedding(query_text)
        else:
            query_embedding = self.embedding_service.get_embedding(query_text)
        return self.search(query_embedding, top_k, chapter_filter)

    def search(
        self,
        query_embedding: List[float],
        top_k: int = 5,
        chapter_filter: Optional[str] = None,
    ) -> List[Dict]:
        """
        Search using embedding vector.
        Note: chapter_filter is ignored since Qdrant free tier doesn't support payload indexing.
        """
        # Skip chapter filtering - not indexed in Qdrant free tier
        filter_condition = None

        # Use query_points with vector (new qdrant-client API)
        results = self.client.query_points(
            collection_name=self.collection_name,
            query=query_embedding,
            limit=top_k,
            query_filter=filter_condition,
            with_payload=True,
        )

        return [
            {
                "text": hit.payload.get("text", ""),
                "chapter": hit.payload.get("chapter", ""),
                "section": hit.payload.get("section", ""),
                "score": hit.score,
            }
            for hit in results.points
        ]

    def delete_collection(self):
        """Delete the entire collection (for re-indexing)."""
        self.client.delete_collection(collection_name=self.collection_name)

    def get_collection_info(self) -> Dict:
        """Get collection statistics."""
        try:
            info = self.client.get_collection(collection_name=self.collection_name)
            return {
                "name": self.collection_name,
                "points_count": getattr(info, 'points_count', None),
                "status": info.status.value if hasattr(info, 'status') else "unknown",
            }
        except Exception as e:
            return {"error": str(e)}

    def health_check(self) -> bool:
        """Check if Qdrant is accessible."""
        try:
            self.client.get_collections()
            return True
        except Exception:
            return False

    @staticmethod
    def _generate_id(text: str, chapter: str) -> int:
        """Generate deterministic ID from content."""
        content = f"{chapter}:{text[:200]}"
        hash_obj = hashlib.md5(content.encode())
        return int(hash_obj.hexdigest()[:16], 16)


# Singleton instance
_qdrant_service: Optional[QdrantService] = None


def get_qdrant_service() -> QdrantService:
    """Get or create Qdrant service singleton."""
    global _qdrant_service
    if _qdrant_service is None:
        _qdrant_service = QdrantService()
    return _qdrant_service
