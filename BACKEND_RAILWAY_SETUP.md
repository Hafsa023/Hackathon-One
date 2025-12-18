# Backend Deployment on Railway - Quick Guide

## Project Structure
- **Frontend**: Deployed on Vercel ✅
- **Backend**: Deploying on Railway (this guide)

---

## Step 1: Set Railway Root Path (IMPORTANT!)

Railway needs to know your backend is in the `/backend` folder:

1. Go to your Railway project dashboard
2. Click on your service
3. Go to **"Settings"** tab
4. Find **"Root Directory"** setting
5. Change from `/` to `/backend`
6. Click **"Save"**

---

## Step 2: Add Environment Variables

In Railway dashboard, go to **"Variables"** tab and add:

```bash
# Required - AI Chat
QWEN_API_KEY=your-openrouter-api-key
QWEN_BASE_URL=https://openrouter.ai/api/v1
QWEN_CHAT_MODEL=qwen/qwen-2.5-72b-instruct

# Required - Vector Database
QDRANT_URL=https://your-cluster.cloud.qdrant.io:6333
QDRANT_API_KEY=your-qdrant-api-key
QDRANT_COLLECTION_NAME=book_chatbot

# Required - PostgreSQL
DATABASE_URL=postgresql://user:password@host.neon.tech/chatbot?sslmode=require

# Optional - CORS (your Vercel URL)
CORS_ORIGINS=https://your-vercel-app.vercel.app,http://localhost:3000

# Optional - Configuration
CHUNK_SIZE=400
CHUNK_OVERLAP=50
TOP_K_RESULTS=5
MAX_CONTEXT_LENGTH=4000
```

**Important**: Update `CORS_ORIGINS` with your actual Vercel URL!

---

## Step 3: Deploy!

After setting root directory and environment variables:

1. Railway will automatically redeploy
2. Wait 2-3 minutes for build
3. Check **"Deployments"** tab for progress

---

## Step 4: Get Your Backend URL

1. In Railway, go to **"Settings"** tab
2. Scroll to **"Networking"** section
3. Click **"Generate Domain"** (if not exists)
4. Copy the URL (e.g., `https://your-backend.up.railway.app`)

---

## Step 5: Update Vercel Frontend

Your Vercel frontend needs to know where the backend is:

### Option A: Update in Vercel Dashboard
1. Go to your Vercel project
2. Go to **"Settings"** → **"Environment Variables"**
3. Add: `REACT_APP_API_URL=https://your-backend.up.railway.app`
4. Redeploy

### Option B: Update in Code
Edit your frontend API configuration file to point to the Railway backend URL.

---

## Verify Everything Works

1. **Test Backend API**: Visit `https://your-backend.up.railway.app/docs`
   - Should see FastAPI documentation

2. **Test Frontend**: Visit your Vercel URL
   - Chatbot should be able to connect to backend

3. **Test Chat**: Try asking a question
   - Should get responses from the AI

---

## Troubleshooting

### Backend won't start?
- ✅ Check **"Root Directory"** is set to `/backend`
- ✅ Check all environment variables are set
- ✅ Look at **"Logs"** tab for errors
- ✅ Verify API keys are valid

### CORS errors in frontend?
- ✅ Add your Vercel URL to `CORS_ORIGINS` in Railway
- ✅ Format: `https://your-app.vercel.app` (no trailing slash)
- ✅ Can use `*` for testing (not recommended for production)

### 404 errors?
- ✅ Make sure **"Root Directory"** is `/backend` not `/`
- ✅ Check Railway is using the correct start command

---

## What's Next?

After successful deployment:

1. **Populate Vector Database** with your book content
2. **Test all chatbot features**
3. **Monitor usage** in service dashboards
4. **Set up monitoring/alerts** (optional)

---

## Quick Reference

- **Railway Dashboard**: https://railway.app/dashboard
- **OpenRouter Usage**: https://openrouter.ai/activity
- **Qdrant Dashboard**: https://cloud.qdrant.io/
- **Neon Dashboard**: https://console.neon.tech/

---

## Need the Full Setup Guide?

If you haven't set up OpenRouter, Qdrant, or Neon yet, see:
- `RAILWAY_SETUP_GUIDE.md` - Complete service setup
- `DEPLOYMENT_CHECKLIST.md` - Step-by-step checklist

---

**Good luck! 🚀**
