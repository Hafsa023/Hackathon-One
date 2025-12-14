# Physical AI Book - RAG Chatbot Backend

A FastAPI-based backend that provides RAG (Retrieval-Augmented Generation) capabilities for the Physical AI textbook chatbot.

## Architecture Overview

```
┌─────────────────────────────────────────────────────────────────┐
│                      Docusaurus Frontend                        │
│                    (Chat Widget Component)                      │
└─────────────────────────────────────────────────────────────────┘
                              │
                              ▼
┌─────────────────────────────────────────────────────────────────┐
│                      FastAPI Backend                            │
│  ┌─────────────┐  ┌─────────────┐  ┌─────────────────────────┐  │
│  │ /chat       │  │ /health     │  │ /history/{session_id}   │  │
│  │ endpoint    │  │ endpoint    │  │ endpoint                │  │
│  └─────────────┘  └─────────────┘  └─────────────────────────┘  │
└─────────────────────────────────────────────────────────────────┘
          │                                      │
          ▼                                      ▼
┌───────────────────┐              ┌───────────────────────────┐
│   Qdrant Cloud    │              │   Neon Serverless Postgres│
│  (Vector Search)  │              │    (Chat History)         │
└───────────────────┘              └───────────────────────────┘
          │
          ▼
┌───────────────────┐
│   OpenAI API      │
│  (Embeddings +    │
│   Completions)    │
└───────────────────┘
```

## Features

- **RAG Pipeline**: Retrieves relevant book content and generates accurate answers
- **Selected Text Mode**: Answer questions based on user-highlighted text
- **Chat History**: Persistent conversation storage in Postgres
- **Session Tracking**: Analytics and usage monitoring
- **Health Checks**: Service status monitoring

## Quick Start

### 1. Prerequisites

- Python 3.10+
- Qdrant Cloud account (free tier available)
- Neon Serverless Postgres account (free tier available)
- OpenAI API key

### 2. Installation

```bash
cd backend

# Create virtual environment
python -m venv venv

# Activate virtual environment
# Windows:
venv\Scripts\activate
# macOS/Linux:
source venv/bin/activate

# Install dependencies
pip install -r requirements.txt
```

### 3. Configuration

Copy the example environment file and fill in your credentials:

```bash
cp .env.example .env
```

Edit `.env` with your API keys:

```env
# OpenAI
OPENAI_API_KEY=sk-your-key-here

# Qdrant Cloud
QDRANT_URL=https://your-cluster.qdrant.io:6333
QDRANT_API_KEY=your-qdrant-api-key

# Neon Postgres
DATABASE_URL=postgresql://user:pass@host/db?sslmode=require
```

### 4. Initialize Database

The database tables are created automatically on startup.

### 5. Ingest Book Content

```bash
# From the backend directory
python -m scripts.ingest --docs-path ../docs

# Force re-index (deletes existing data)
python -m scripts.ingest --docs-path ../docs --force
```

### 6. Run the Server

```bash
# Development
uvicorn app.main:app --reload --host 0.0.0.0 --port 8000

# Production
uvicorn app.main:app --host 0.0.0.0 --port 8000 --workers 4
```

## API Endpoints

### POST /chat

Send a question and receive an answer from the book content.

**Request:**
```json
{
  "question": "What is embodied intelligence?",
  "selected_text": null,
  "session_id": "optional-session-id",
  "chapter": "optional-chapter-context"
}
```

**Response:**
```json
{
  "answer": "Embodied intelligence refers to...",
  "sources": [
    {
      "chapter": "Introduction to Physical AI",
      "section": "Core Concepts",
      "relevance_score": 0.89
    }
  ],
  "session_id": "abc123",
  "timestamp": "2025-01-15T10:30:00Z"
}
```

### GET /health

Check service health status.

**Response:**
```json
{
  "status": "healthy",
  "qdrant_connected": true,
  "database_connected": true,
  "version": "1.0.0"
}
```

### GET /history/{session_id}

Retrieve chat history for a session.

**Response:**
```json
[
  {
    "id": 1,
    "question": "What is ROS 2?",
    "answer": "ROS 2 is...",
    "chapter": "ros2/fundamentals",
    "created_at": "2025-01-15T10:30:00Z"
  }
]
```

### GET /collection/info

Get Qdrant collection statistics.

**Response:**
```json
{
  "name": "physical_ai_book",
  "vectors_count": 150,
  "points_count": 150,
  "status": "green"
}
```

## Project Structure

```
backend/
├── app/
│   ├── __init__.py
│   ├── main.py           # FastAPI application
│   ├── config.py         # Configuration settings
│   ├── models.py         # Pydantic models
│   ├── database.py       # Neon Postgres connection
│   ├── qdrant_client.py  # Qdrant vector operations
│   ├── openai_service.py # OpenAI embeddings & chat
│   └── chat_service.py   # RAG pipeline orchestration
├── scripts/
│   ├── __init__.py
│   └── ingest.py         # Book content ingestion
├── requirements.txt
├── .env.example
└── README.md
```

## Environment Variables

| Variable | Description | Default |
|----------|-------------|---------|
| `OPENAI_API_KEY` | OpenAI API key | Required |
| `OPENAI_EMBEDDING_MODEL` | Embedding model | `text-embedding-3-small` |
| `OPENAI_CHAT_MODEL` | Chat model | `gpt-4o-mini` |
| `QDRANT_URL` | Qdrant Cloud URL | Required |
| `QDRANT_API_KEY` | Qdrant API key | Required |
| `QDRANT_COLLECTION_NAME` | Collection name | `physical_ai_book` |
| `DATABASE_URL` | Postgres connection string | Required |
| `CHUNK_SIZE` | Words per chunk | `400` |
| `CHUNK_OVERLAP` | Overlap between chunks | `50` |
| `TOP_K_RESULTS` | Chunks to retrieve | `5` |
| `MAX_CONTEXT_LENGTH` | Max context tokens | `4000` |
| `CORS_ORIGINS` | Allowed origins | `*` |

## Deployment

### Docker

```dockerfile
FROM python:3.11-slim

WORKDIR /app
COPY requirements.txt .
RUN pip install --no-cache-dir -r requirements.txt

COPY app/ app/
COPY scripts/ scripts/

EXPOSE 8000
CMD ["uvicorn", "app.main:app", "--host", "0.0.0.0", "--port", "8000"]
```

### Railway / Render / Fly.io

1. Connect your repository
2. Set environment variables
3. Deploy with the start command: `uvicorn app.main:app --host 0.0.0.0 --port $PORT`

## Testing

```bash
# Run tests
pytest

# With coverage
pytest --cov=app
```

## Troubleshooting

### "Cannot connect to Qdrant"
- Verify `QDRANT_URL` and `QDRANT_API_KEY` are correct
- Check if your IP is whitelisted in Qdrant Cloud

### "Database connection failed"
- Verify `DATABASE_URL` format
- Ensure `?sslmode=require` is included for Neon

### "OpenAI API error"
- Check API key validity
- Verify you have sufficient credits

### "No results found"
- Run the ingestion script first
- Check collection info endpoint for vector count
