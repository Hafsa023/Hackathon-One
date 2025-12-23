---
id: 003
title: RAG Chatbot Implementation for Physical AI Book
stage: general
date: 2025-12-14
surface: agent
model: claude-sonnet-4-5-20250929
feature: rag-chatbot
branch: main
user: system
command: direct-implementation
labels: ["rag", "chatbot", "fastapi", "qdrant", "openai", "docusaurus"]
links:
  spec: null
  ticket: null
  adr: null
  pr: null
files:
  - backend/app/__init__.py
  - backend/app/config.py
  - backend/app/models.py
  - backend/app/database.py
  - backend/app/qdrant_client.py
  - backend/app/openai_service.py
  - backend/app/chat_service.py
  - backend/app/main.py
  - backend/scripts/ingest.py
  - backend/scripts/__init__.py
  - backend/scripts/schema.sql
  - backend/requirements.txt
  - backend/.env.example
  - backend/README.md
  - backend/Dockerfile
  - backend/.dockerignore
  - src/components/ChatWidget/index.tsx
  - src/components/ChatWidget/styles.module.css
  - src/config/chatConfig.ts
  - src/theme/Root.tsx (modified)
  - docker-compose.yml
  - CHATBOT_SETUP.md
tests:
  - Manual API testing via curl endpoints
  - Frontend integration testing via browser
---

## Prompt

Project: Physical AI Textbook — RAG Chatbot Integration
Goal: Build and embed a **Retrieval-Augmented Generation (RAG) chatbot** into the published Docusaurus book website. The chatbot must answer questions using the **book content only**, including **selected text by the user**.

Key requirements:
1. FastAPI backend with /chat endpoint
2. Qdrant Cloud for vector storage
3. Neon Serverless Postgres for chat history
4. React chat widget for Docusaurus
5. Selected text highlighting feature
6. Strict answer sourcing from book content only

## Response snapshot

Implemented a complete RAG chatbot system with the following components:

**Backend (FastAPI):**
- `/chat` endpoint accepting question, selected_text, session_id, chapter
- `/health` endpoint for service monitoring
- `/history/{session_id}` for chat history retrieval
- `/collection/info` for Qdrant statistics

**Services:**
- `OpenAIService`: Embeddings (text-embedding-3-small) and chat completions (gpt-4o-mini)
- `QdrantService`: Vector storage, search, and management
- `ChatService`: RAG pipeline orchestration
- Database layer with SQLAlchemy for Neon Postgres

**Ingestion Script:**
- Markdown processing with frontmatter extraction
- Smart chunking (~400 words with 50-word overlap)
- Batch embedding generation
- Qdrant vector upload

**Frontend:**
- ChatWidget React component with floating bubble UI
- Text selection detection hook
- Session persistence via localStorage
- Responsive design with dark mode support
- Source citations in responses

**Configuration:**
- Environment-based settings via pydantic-settings
- Docker and docker-compose support
- Comprehensive documentation

## Outcome

- ✅ Impact: Complete RAG chatbot system ready for deployment
- 🧪 Tests: Manual testing via API endpoints and browser
- 📁 Files: 22 files created/modified across backend and frontend
- 🔁 Next prompts: Deploy backend, configure production API URL, run ingestion
- 🧠 Reflection: System designed for strict content-grounding to prevent hallucinations

## Evaluation notes (flywheel)

- Failure modes observed: None during implementation
- Graders run and results (PASS/FAIL): N/A - implementation task
- Prompt variant (if applicable): N/A
- Next experiment (smallest change to try): Test with real API keys and verify end-to-end flow
