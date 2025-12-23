"""
FastAPI backend for Physical AI Book RAG Chatbot.
Provides endpoints for chat, health checks, and admin operations.
"""

from fastapi import FastAPI, HTTPException, Depends
from fastapi.middleware.cors import CORSMiddleware
from fastapi.responses import StreamingResponse
from sqlalchemy.orm import Session
from sqlalchemy import text
from datetime import datetime
from contextlib import asynccontextmanager
from typing import Optional

from .config import get_settings
from .models import (
    ChatRequest,
    ChatResponse,
    HealthResponse,
    Source,
)
from .database import (
    init_db,
    get_db_session,
    save_chat_history,
    get_session_history,
    log_analytics_event,
    is_db_enabled,
)
from .qdrant_client import get_qdrant_service
from .chat_service import get_chat_service

settings = get_settings()


@asynccontextmanager
async def lifespan(app: FastAPI):
    """Application lifespan events."""
    # Startup
    print("=" * 50)
    print("Starting Physical AI Book Chatbot Backend...")
    print("=" * 50)

    # Initialize database (optional)
    init_db()

    # Initialize Qdrant
    try:
        qdrant = get_qdrant_service()
        qdrant.ensure_collection_exists()
        print("Qdrant connection: OK")
    except Exception as e:
        print(f"Qdrant connection: FAILED - {e}")

    print("=" * 50)
    print("Backend initialized - visit http://localhost:8000/docs")
    print("=" * 50)
    yield
    # Shutdown
    print("Shutting down...")


app = FastAPI(
    title="Physical AI Book Chatbot API",
    description="RAG-powered chatbot for the Physical AI textbook using Qwen",
    version="1.0.0",
    lifespan=lifespan,
)

# Configure CORS
app.add_middleware(
    CORSMiddleware,
    allow_origins=settings.cors_origins.split(","),
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)


@app.get("/", tags=["Root"])
async def root():
    """Root endpoint with API info."""
    return {
        "name": "Physical AI Book Chatbot API",
        "version": "1.0.0",
        "model": "Qwen via OpenRouter",
        "docs": "/docs",
    }


@app.get("/health", response_model=HealthResponse, tags=["Health"])
async def health_check(db: Optional[Session] = Depends(get_db_session)):
    """Check the health of all services."""
    qdrant = get_qdrant_service()

    # Check Qdrant
    qdrant_ok = qdrant.health_check()

    # Check Database (optional)
    db_ok = False
    if db is not None:
        try:
            db.execute(text("SELECT 1"))
            db_ok = True
        except Exception:
            db_ok = False
    else:
        db_ok = False  # Database not configured

    # Status is healthy if Qdrant works (database is optional)
    status = "healthy" if qdrant_ok else "degraded"

    return HealthResponse(
        status=status,
        qdrant_connected=qdrant_ok,
        database_connected=db_ok,
    )


@app.post("/chat", response_model=ChatResponse, tags=["Chat"])
async def chat(
    request: ChatRequest,
    db: Optional[Session] = Depends(get_db_session),
):
    """
    Process a chat question and return an answer.

    The chatbot uses RAG (Retrieval-Augmented Generation) to:
    1. If selected_text is provided: Answer based on that text only
    2. Otherwise: Search the book content in Qdrant and use relevant chunks

    Answers are strictly limited to book content. If information is not found,
    the chatbot will respond with "I cannot find this information in the book."
    """
    try:
        print(f"[CHAT] Received request: question='{request.question[:50]}...', session_id={request.session_id}")
        chat_service = get_chat_service()

        result = await chat_service.process_question(
            question=request.question,
            selected_text=request.selected_text,
            session_id=request.session_id,
            chapter=request.chapter,
        )
        print(f"[CHAT] Generated answer: {result['answer'][:50]}...")

        # Save to chat history (if database is configured)
        source_chapters = [s.chapter for s in result["sources"]]
        save_chat_history(
            db=db,
            session_id=result["session_id"],
            question=request.question,
            answer=result["answer"],
            chapter=request.chapter,
            sources=source_chapters,
        )

        # Log analytics event (if database is configured)
        log_analytics_event(
            db=db,
            event_type="chat_question",
            session_id=result["session_id"],
            chapter=request.chapter,
            event_data={
                "has_selected_text": bool(request.selected_text),
                "question_length": len(request.question),
            },
        )

        return ChatResponse(
            answer=result["answer"],
            sources=result["sources"],
            session_id=result["session_id"],
            timestamp=datetime.utcnow(),
        )

    except Exception as e:
        import traceback
        error_trace = traceback.format_exc()
        print(f"[CHAT ERROR] {str(e)}")
        print(error_trace)
        # Return detailed error for debugging
        raise HTTPException(status_code=500, detail=f"{str(e)}\n\nTraceback:\n{error_trace}")


@app.post("/chat/stream", tags=["Chat"])
async def chat_stream(
    request: ChatRequest,
    db: Optional[Session] = Depends(get_db_session),
):
    """
    Process a chat question and stream the answer using Server-Sent Events.

    The response is streamed as SSE with three message types:
    - metadata: Contains sources and session_id
    - content: Contains answer text chunks
    - done: Signals completion with full answer

    The chatbot uses RAG (Retrieval-Augmented Generation) to:
    1. If selected_text is provided: Answer based on that text only
    2. Otherwise: Search the book content in Qdrant and use relevant chunks
    """
    try:
        print(f"[CHAT STREAM] Received request: question='{request.question[:50]}...', session_id={request.session_id}")
        chat_service = get_chat_service()

        async def generate():
            full_answer = ""
            session_id = request.session_id

            async for chunk in chat_service.process_question_stream(
                question=request.question,
                selected_text=request.selected_text,
                session_id=request.session_id,
                chapter=request.chapter,
            ):
                # Parse the SSE data to capture full answer and session_id
                if 'data: ' in chunk:
                    import json
                    data_str = chunk.replace('data: ', '').strip()
                    if data_str:
                        try:
                            data = json.loads(data_str)
                            if data.get('type') == 'metadata':
                                session_id = data.get('session_id', session_id)
                            elif data.get('type') == 'done':
                                full_answer = data.get('full_answer', '')
                        except json.JSONDecodeError:
                            pass
                yield chunk

            # Save to chat history after streaming completes (if database is configured)
            if db and full_answer:
                try:
                    save_chat_history(
                        db=db,
                        session_id=session_id or request.session_id or "unknown",
                        question=request.question,
                        answer=full_answer,
                        chapter=request.chapter,
                        sources=[],
                    )
                    log_analytics_event(
                        db=db,
                        event_type="chat_question_stream",
                        session_id=session_id or request.session_id or "unknown",
                        chapter=request.chapter,
                        event_data={
                            "has_selected_text": bool(request.selected_text),
                            "question_length": len(request.question),
                        },
                    )
                except Exception as e:
                    print(f"[CHAT STREAM] Failed to save history: {e}")

        return StreamingResponse(
            generate(),
            media_type="text/event-stream",
            headers={
                "Cache-Control": "no-cache",
                "Connection": "keep-alive",
                "X-Accel-Buffering": "no",
            },
        )

    except Exception as e:
        import traceback
        error_trace = traceback.format_exc()
        print(f"[CHAT STREAM ERROR] {str(e)}")
        print(error_trace)
        raise HTTPException(status_code=500, detail=f"{str(e)}\n\nTraceback:\n{error_trace}")


@app.get("/history/{session_id}", tags=["History"])
async def get_history(
    session_id: str,
    limit: int = 10,
    db: Optional[Session] = Depends(get_db_session),
):
    """Get chat history for a session."""
    if db is None:
        return {"message": "Database not configured - history unavailable", "history": []}

    history = get_session_history(db, session_id, limit)
    return [
        {
            "id": h.id,
            "question": h.question,
            "answer": h.answer,
            "chapter": h.chapter,
            "created_at": h.created_at,
        }
        for h in history
    ]


@app.get("/collection/info", tags=["Admin"])
async def collection_info():
    """Get Qdrant collection statistics."""
    qdrant = get_qdrant_service()
    return qdrant.get_collection_info()


if __name__ == "__main__":
    import uvicorn
    uvicorn.run(app, host="0.0.0.0", port=8000)
