# How to Deploy Backend to Railway - Step by Step

## Prerequisites
✅ Railway CLI installed (already done - v4.15.0)
✅ Backend code ready with all config files
✅ Environment variables from .env file

## Step-by-Step Deployment Guide

### Step 1: Open Command Prompt or PowerShell

Open a new terminal window (Command Prompt, PowerShell, or Git Bash)

### Step 2: Navigate to Backend Directory

```bash
cd D:\Book_chatbot\backend
```

### Step 3: Login to Railway

```bash
railway login
```

**What happens:**
- A browser window will open
- Click "Login with GitHub" or "Login with Email"
- Authorize Railway CLI
- Return to your terminal - you should see "Logged in successfully"

### Step 4: Create a New Railway Project

```bash
railway init
```

**What happens:**
- You'll be asked to name your project
- Enter: `book-chatbot-backend`
- Railway creates a new project linked to this directory

**Expected output:**
```
✓ Created project: book-chatbot-backend
✓ Linked to project
```

### Step 5: Verify Configuration Files

Check that these files exist in your backend directory:
```bash
dir railway.json
dir nixpacks.toml
dir Procfile
dir requirements.txt
```

All should show "File found" or list the files.

### Step 6: Set Environment Variables

Copy and paste ALL these commands (one at a time or all together):

```bash
railway variables set QWEN_API_KEY="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"

railway variables set QWEN_BASE_URL="https://openrouter.ai/api/v1"

railway variables set QWEN_CHAT_MODEL="qwen/qwen-2.5-72b-instruct"

railway variables set QDRANT_URL="https://38e65eab-cb19-47f2-a63a-c792711d36fe.europe-west3-0.gcp.cloud.qdrant.io:6333"

railway variables set QDRANT_API_KEY="eyJhbGciOiJIUzI1NiIsInR5cCI6IkpXVCJ9.eyJhY2Nlc3MiOiJtIn0.rUAwVZWmdTCS4knmEoiUyZx9xSY2p3DndkjtL97Yaj0"

railway variables set QDRANT_COLLECTION_NAME="book_chatbot"

railway variables set CHUNK_SIZE="400"

railway variables set CHUNK_OVERLAP="50"

railway variables set TOP_K_RESULTS="5"

railway variables set MAX_CONTEXT_LENGTH="4000"

railway variables set CORS_ORIGINS="http://localhost:3000,http://localhost:8000"
```

**Expected output for each:**
```
✓ Variable QWEN_API_KEY set
✓ Variable QWEN_BASE_URL set
...
```

### Step 7: Deploy to Railway

```bash
railway up
```

**What happens:**
- Railway uploads your code
- Installs Python 3.11
- Installs dependencies from requirements.txt
- Starts your FastAPI server
- This takes 2-3 minutes

**Expected output:**
```
✓ Build successful
✓ Deployment live
```

### Step 8: Get Your Public URL

```bash
railway domain
```

If no domain exists yet, generate one:
```bash
railway domain generate
```

**Expected output:**
```
https://book-chatbot-backend-production-xxxx.up.railway.app
```

**🎉 COPY THIS URL - You'll need it!**

### Step 9: Test Your Deployment

Open in browser:
```
https://your-railway-url.railway.app/docs
```

Or test with curl:
```bash
curl https://your-railway-url.railway.app/health
```

Expected response:
```json
{
  "status": "healthy",
  "qdrant_connected": true,
  "database_connected": false
}
```

### Step 10: Update CORS (IMPORTANT!)

Now that you have your Railway URL, add it to CORS:

```bash
railway variables set CORS_ORIGINS="https://your-railway-url.railway.app,http://localhost:3000,http://localhost:8000"
```

Replace `your-railway-url.railway.app` with your actual URL from Step 8.

**Railway will automatically redeploy when you change variables.**

---

## Quick Command Summary

```bash
# 1. Navigate to backend
cd D:\Book_chatbot\backend

# 2. Login
railway login

# 3. Create project
railway init

# 4. Set variables (copy all from Step 6 above)

# 5. Deploy
railway up

# 6. Get URL
railway domain

# 7. Update CORS with your URL
railway variables set CORS_ORIGINS="https://your-url.railway.app,http://localhost:3000"
```

---

## Common Issues & Solutions

### Issue 1: "npm run build" error
**Solution:** Make sure you're in the `backend` directory, not the root directory.

### Issue 2: "Unauthorized. Please login"
**Solution:** Run `railway login` again and complete browser authentication.

### Issue 3: Build fails with Python errors
**Solution:** Check that all files exist:
- railway.json
- nixpacks.toml
- Procfile
- requirements.txt

### Issue 4: Deployment succeeds but API doesn't work
**Solution:**
1. Check environment variables are set: `railway variables`
2. Check logs: `railway logs`
3. Verify Qdrant and OpenRouter API keys are correct

### Issue 5: CORS errors when testing from frontend
**Solution:** Update CORS_ORIGINS to include your Railway URL (Step 10)

---

## Useful Railway Commands

```bash
# View all your projects
railway list

# Check current project status
railway status

# View real-time logs
railway logs

# View environment variables
railway variables

# Open Railway dashboard in browser
railway open

# Redeploy (if needed)
railway up

# Check which project you're linked to
railway status
```

---

## What Happens During Deployment?

1. **Upload**: Railway receives your code
2. **Detect**: Railway detects Python project (via requirements.txt)
3. **Build**: Installs Python 3.11 and dependencies
4. **Start**: Runs `uvicorn app.main:app --host 0.0.0.0 --port $PORT`
5. **Expose**: Gives you a public HTTPS URL
6. **Monitor**: Keeps your service running 24/7

---

## Cost

Railway offers:
- **Hobby Plan**: $5/month usage included (enough for testing)
- **Pro Plan**: $20/month with $20 credit

Your chatbot backend should use minimal resources.

---

## Next Steps After Deployment

1. ✅ Copy your Railway URL
2. ✅ Update CORS settings
3. ✅ Test API endpoints at `/docs`
4. ✅ Update frontend configuration with Railway URL
5. ✅ Run document ingestion: POST to `/ingest` endpoint
6. ✅ Test chatbot functionality

---

**Ready? Start with Step 1 and follow each step carefully!**
