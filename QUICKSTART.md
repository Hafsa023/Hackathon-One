# Quick Start Guide

## The Problem You're Experiencing

If you're seeing `ERR_CONNECTION_REFUSED`, it means the **backend server is not running**. The frontend (Docusaurus) is trying to connect to `http://localhost:8000` but nothing is listening on that port.

## Solution: Start Both Services

You need to run **BOTH** the backend and frontend servers:

### Option 1: Start Everything at Once (Recommended)

**Double-click** `START_ALL.bat` in the project root. This will:
- Open a window for the backend server (port 8000)
- Open a window for the frontend (port 3000)

### Option 2: Start Services Separately

#### Start Backend First:
1. Double-click `START_BACKEND.bat` in the project root
2. Wait for the message: "Backend initialized - visit http://localhost:8000/docs"

#### Then Start Frontend:
1. Double-click `START_FRONTEND.bat` in the project root
2. Wait for the browser to open at http://localhost:3000

### Option 3: Manual Start (Terminal)

#### Terminal 1 - Backend:
```bash
cd backend
# Windows
venv\Scripts\activate.bat
# Linux/Mac
source venv/bin/activate

uvicorn app.main:app --reload --host 0.0.0.0 --port 8000
```

#### Terminal 2 - Frontend:
```bash
npm start
```

## Verify It's Working

1. **Backend Health Check**: Open http://localhost:8000/health
   - Should return: `{"status": "healthy", "qdrant_connected": true, ...}`

2. **Backend API Docs**: Open http://localhost:8000/docs
   - Should show FastAPI Swagger UI

3. **Frontend**: Open http://localhost:3000
   - Click the chat bubble in the bottom right
   - Try asking a question

## Common Issues & Solutions

### Issue 1: "ERR_CONNECTION_REFUSED"
**Cause**: Backend server is not running
**Solution**: Start the backend using one of the methods above

### Issue 2: "Port 8000 is already in use"
**Cause**: Another application is using port 8000
**Solution**:
- Find the process: `netstat -ano | findstr :8000`
- Kill it: `taskkill /PID <PID> /F`
- Or change the port in backend startup script

### Issue 3: "ModuleNotFoundError" in backend
**Cause**: Dependencies not installed
**Solution**:
```bash
cd backend
venv\Scripts\activate.bat
pip install -r requirements.txt
```

### Issue 4: Backend starts but crashes immediately
**Cause**: Missing or invalid environment variables
**Solution**:
1. Check that `backend\.env` exists (copy from `.env.example` if not)
2. Verify your API keys are set correctly:
   - QWEN_API_KEY (OpenRouter)
   - QDRANT_URL and QDRANT_API_KEY
3. DATABASE_URL can be left as placeholder - database is optional

### Issue 5: Frontend shows "Unable to connect to server"
**Cause**: Backend is not responding
**Solution**:
1. Verify backend is running (check the backend terminal window)
2. Test backend health: http://localhost:8000/health
3. Check CORS settings in backend\.env (should include http://localhost:3000)

## Development Workflow

For active development:

1. **Keep both terminals open**
   - Backend terminal (port 8000) - auto-reloads on code changes
   - Frontend terminal (port 3000) - auto-reloads on code changes

2. **Watch for errors**
   - Backend errors appear in the backend terminal
   - Frontend errors appear in browser console (F12)

3. **Testing the chat**
   - Open http://localhost:3000
   - Click the chat bubble
   - Ask: "What is embodied intelligence?"

## Architecture Overview

```
Browser (localhost:3000)
    ↓ HTTP Requests
FastAPI Backend (localhost:8000)
    ↓ Vector Search
Qdrant Cloud (vector database)
    ↓ Chat Completion
OpenRouter/Qwen API
```

Both the frontend AND backend must be running for the chat to work.

## Need Help?

1. Check backend logs in the terminal
2. Check browser console (F12) for frontend errors
3. Test backend directly: http://localhost:8000/docs
4. Verify .env configuration in backend folder
