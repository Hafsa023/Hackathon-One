"""
Chat service orchestrating RAG pipeline.
Coordinates between Qdrant retrieval and Qwen generation.
"""

from typing import List, Dict, Optional, Generator, AsyncGenerator
import uuid
import json

from .qdrant_client import get_qdrant_service
from .llm_service import get_llm_service
from .config import get_settings
from .models import Source

settings = get_settings()


class ChatService:
    """Service for handling chat requests with RAG pipeline."""

    def __init__(self):
        self.qdrant = get_qdrant_service()
        self.llm = get_llm_service()

    async def process_question(
        self,
        question: str,
        selected_text: Optional[str] = None,
        session_id: Optional[str] = None,
        chapter: Optional[str] = None,
    ) -> Dict:
        """
        Process a user question through the RAG pipeline.

        Args:
            question: User's question
            selected_text: User-highlighted text (optional)
            session_id: Session ID for tracking
            chapter: Current chapter context

        Returns:
            Dict with answer, sources, and session info
        """
        # Generate session ID if not provided
        if not session_id:
            session_id = str(uuid.uuid4())

        sources: List[Source] = []

        if selected_text:
            # Mode: Answer from selected text only
            context = selected_text
            sources = [Source(
                chapter=chapter or "Selected Text",
                section="User Selection",
                relevance_score=1.0,
            )]
        else:
            # Mode: Retrieve relevant chunks from Qdrant
            context, sources = await self._retrieve_context(question, chapter)

        # Generate answer using Qwen
        answer = self.llm.generate_answer(
            question=question,
            context=context,
            selected_text=selected_text,
        )

        return {
            "answer": answer,
            "sources": sources,
            "session_id": session_id,
        }

    async def process_question_stream(
        self,
        question: str,
        selected_text: Optional[str] = None,
        session_id: Optional[str] = None,
        chapter: Optional[str] = None,
    ) -> AsyncGenerator[str, None]:
        """
        Process a user question through the RAG pipeline with streaming response.

        Args:
            question: User's question
            selected_text: User-highlighted text (optional)
            session_id: Session ID for tracking
            chapter: Current chapter context

        Yields:
            SSE-formatted strings containing answer chunks or metadata
        """
        # Generate session ID if not provided
        if not session_id:
            session_id = str(uuid.uuid4())

        sources: List[Source] = []

        if selected_text:
            # Mode: Answer from selected text only
            context = selected_text
            sources = [Source(
                chapter=chapter or "Selected Text",
                section="User Selection",
                relevance_score=1.0,
            )]
        else:
            # Mode: Retrieve relevant chunks from Qdrant
            context, sources = await self._retrieve_context(question, chapter)

        # Send metadata first (sources and session_id)
        metadata = {
            "type": "metadata",
            "sources": [s.model_dump() for s in sources],
            "session_id": session_id,
        }
        yield f"data: {json.dumps(metadata)}\n\n"

        # Stream the answer
        full_answer = ""
        for chunk in self.llm.generate_answer_stream(
            question=question,
            context=context,
            selected_text=selected_text,
        ):
            full_answer += chunk
            yield f"data: {json.dumps({'type': 'content', 'content': chunk})}\n\n"

        # Send completion signal with full answer for history
        yield f"data: {json.dumps({'type': 'done', 'full_answer': full_answer})}\n\n"

    async def _retrieve_context(
        self,
        question: str,
        chapter_filter: Optional[str] = None,
    ) -> tuple[str, List[Source]]:
        """
        Retrieve relevant context from Qdrant using text search.
        """
        # Search Qdrant using text query (embeddings computed server-side)
        results = self.qdrant.search_with_text(
            query_text=question,
            top_k=settings.top_k_results,
            chapter_filter=chapter_filter,
        )

        if not results:
            return "", []

        # Build context string from results
        context_parts = []
        sources = []

        for result in results:
            # Add to context
            chunk_context = f"[{result['chapter']}]\n{result['text']}\n"
            context_parts.append(chunk_context)

            # Track source
            sources.append(Source(
                chapter=result['chapter'],
                section=result.get('section', ''),
                relevance_score=result['score'],
            ))

        # Combine and truncate if needed
        full_context = "\n---\n".join(context_parts)
        truncated_context = self.llm.truncate_to_token_limit(
            full_context,
            settings.max_context_length,
        )

        return truncated_context, sources


# Singleton instance
_chat_service: Optional[ChatService] = None


def get_chat_service() -> ChatService:
    """Get or create chat service singleton."""
    global _chat_service
    if _chat_service is None:
        _chat_service = ChatService()
    return _chat_service
