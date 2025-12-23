# Physical AI Book - RAG Chatbot Setup Guide

This guide walks you through setting up the RAG chatbot for the Physical AI textbook.

## Table of Contents

1. [Prerequisites](#prerequisites)
2. [Backend Setup](#backend-setup)
3. [Service Configuration](#service-configuration)
4. [Content Ingestion](#content-ingestion)
5. [Frontend Integration](#frontend-integration)
6. [Testing](#testing)
7. [Deployment](#deployment)
8. [Troubleshooting](#troubleshooting)

---

## Prerequisites

Before starting, ensure you have:

- **Node.js 18+** (for Docusaurus)
- **Python 3.10+** (for FastAPI backend)
- **Git** (for version control)

You'll also need accounts for:

- **OpenAI** - For embeddings and chat completions
- **Qdrant Cloud** - For vector storage (free tier available)
- **Neon** - For serverless Postgres (free tier available)

---

## Backend Setup

### Step 1: Create Python Virtual Environment

```bash
cd backend

# Create virtual environment
python -m venv venv

# Activate it
# Windows:
venv\Scripts\activate
# macOS/Linux:
source venv/bin/activate

# Install dependencies
pip install -r requirements.txt
```

### Step 2: Configure Environment Variables

```bash
# Copy the example file
cp .env.example .env

# Edit .env with your credentials
```

---

## Service Configuration

### OpenAI Setup

1. Go to [platform.openai.com](https://platform.openai.com)
2. Navigate to API Keys
3. Create a new API key
4. Add to `.env`:
   ```
   OPENAI_API_KEY=sk-your-key-here
   ```

### Qdrant Cloud Setup

1. Go to [cloud.qdrant.io](https://cloud.qdrant.io)
2. Create a free cluster
3. Copy the cluster URL and API key
4. Add to `.env`:
   ```
   QDRANT_URL=https://your-cluster-id.us-east4-0.gcp.cloud.qdrant.io:6333
   QDRANT_API_KEY=your-api-key
   ```

### Neon Postgres Setup

1. Go to [console.neon.tech](https://console.neon.tech)
2. Create a new project
3. Copy the connection string
4. Add to `.env`:
   ```
   DATABASE_URL=postgresql://user:pass@ep-xxx.us-east-2.aws.neon.tech/neondb?sslmode=require
   ```

---

## Content Ingestion

### Ingest Book Content into Qdrant

The ingestion script processes all markdown files in the `docs/` directory:

```bash
cd backend

# Activate virtual environment if not already active
venv\Scripts\activate  # Windows
source venv/bin/activate  # macOS/Linux

# Run ingestion
python -m scripts.ingest --docs-path ../docs

# Force re-index (clears existing data first)
python -m scripts.ingest --docs-path ../docs --force
```

**What happens during ingestion:**

1. Reads all `.md` and `.mdx` files from docs
2. Extracts YAML frontmatter for metadata
3. Cleans markdown syntax
4. Splits content into ~400-word chunks with overlap
5. Generates embeddings via OpenAI
6. Uploads vectors to Qdrant

**Expected output:**
```
Starting ingestion from: D:\Book_chatbot\docs
Found 14 markdown files
Processing: intro.md
  - Generated 12 chunks
Processing: embodied-intelligence.md
  - Generated 18 chunks
...
Total chunks: 156
Generating embeddings...
  Embedding batch 1/2
  Embedding batch 2/2
Uploading to Qdrant...
Uploaded 156 chunks to Qdrant

INGESTION COMPLETE
Files processed: 14
Total chunks: 156
Chunks uploaded: 156

Collection info:
  name: physical_ai_book
  vectors_count: 156
  points_count: 156
  status: green
```

---

## Frontend Integration

### Step 1: Update Chat Widget Configuration

Edit `src/config/chatConfig.ts`:

```typescript
export const CHAT_CONFIG = {
  // Development
  API_BASE_URL: 'http://localhost:8000',

  // Production (update this)
  // API_BASE_URL: 'https://your-backend.com',
};
```

### Step 2: The Chat Widget is Already Integrated

The `ChatWidget` component is already added to `src/theme/Root.tsx`:

```tsx
import ChatWidget from '@site/src/components/ChatWidget';

export default function Root({ children }) {
  const isDocsPage = location.pathname.startsWith('/docs');

  return (
    <>
      {children}
      {isDocsPage && <ChatWidget />}
    </>
  );
}
```

### Step 3: Build and Preview

```bash
# From the root directory
npm run build
npm run serve
```

---

## Testing

### Test Backend API

```bash
# Start the backend server
cd backend
uvicorn app.main:app --reload

# In another terminal, test endpoints:

# Health check
curl http://localhost:8000/health

# Test chat
curl -X POST http://localhost:8000/chat \
  -H "Content-Type: application/json" \
  -d '{"question": "What is embodied intelligence?"}'

# Check collection info
curl http://localhost:8000/collection/info
```

### Test Chat Widget

1. Start the backend: `uvicorn app.main:app --reload`
2. Start Docusaurus: `npm start`
3. Navigate to any docs page
4. Click the chat bubble in the bottom-right
5. Ask a question about the book content

### Test Selected Text Feature

1. Navigate to a docs page
2. Highlight some text with your mouse
3. Notice the yellow indicator on the chat bubble
4. Open the chat and ask a question
5. The answer should reference the selected text

---

## Deployment

### Backend Deployment Options

#### Option A: Railway

1. Connect your GitHub repository
2. Set environment variables in Railway dashboard
3. Deploy with:
   ```
   Start command: uvicorn app.main:app --host 0.0.0.0 --port $PORT
   ```

#### Option B: Render

1. Create a new Web Service
2. Connect repository
3. Set environment variables
4. Build command: `pip install -r requirements.txt`
5. Start command: `uvicorn app.main:app --host 0.0.0.0 --port $PORT`

#### Option C: Docker

```dockerfile
# backend/Dockerfile
FROM python:3.11-slim

WORKDIR /app
COPY requirements.txt .
RUN pip install --no-cache-dir -r requirements.txt

COPY app/ app/
COPY scripts/ scripts/

EXPOSE 8000
CMD ["uvicorn", "app.main:app", "--host", "0.0.0.0", "--port", "8000"]
```

### Frontend Deployment

Update `src/config/chatConfig.ts` with your production backend URL, then deploy as normal:

```bash
npm run build
# Deploy the build/ folder to your hosting provider
```

---

## Troubleshooting

### "CORS Error in Browser Console"

Update `CORS_ORIGINS` in your `.env`:

```
CORS_ORIGINS=http://localhost:3000,https://your-docusaurus-site.com
```

### "Cannot connect to Qdrant"

1. Verify credentials in `.env`
2. Check if cluster is running in Qdrant dashboard
3. Ensure your IP isn't blocked

### "Database connection failed"

1. Check connection string format
2. Ensure `?sslmode=require` is included
3. Verify Neon project is active

### "No relevant results found"

1. Run ingestion script: `python -m scripts.ingest --docs-path ../docs`
2. Check collection info: `curl http://localhost:8000/collection/info`
3. Verify chunks were uploaded

### "Chat widget not appearing"

1. Ensure you're on a `/docs/*` page
2. Check browser console for errors
3. Verify `src/theme/Root.tsx` includes the ChatWidget

### "Selected text not detected"

1. Text must be at least 10 characters
2. Selection works on docs pages only
3. Check browser console for JavaScript errors

---

## Test Workflow Example

Here's a complete test scenario:

1. **User opens book chapter**: Goes to `/docs/foundations/embodied-intelligence`

2. **User asks general question**:
   - Clicks chat bubble
   - Types: "What are the three pillars of embodied AI?"
   - Gets answer with sources from the book

3. **User highlights text and asks follow-up**:
   - Selects paragraph about "morphological computation"
   - Yellow indicator appears on chat bubble
   - Types: "Can you explain this in simpler terms?"
   - Gets answer specifically about the selected text

4. **Answer validation**:
   - Response includes source chapter references
   - If info not in book: "I cannot find this information in the book."

---

## Architecture Summary

```
User → Docusaurus (Chat Widget)
         ↓
    FastAPI Backend
         ↓
    ┌────┴────┐
    ↓         ↓
Qdrant    OpenAI
(search)  (generate)
    ↓
  Answer → User
    ↓
Neon Postgres
(store history)
```

The chatbot strictly answers from book content, with no hallucination outside the retrieved context.
