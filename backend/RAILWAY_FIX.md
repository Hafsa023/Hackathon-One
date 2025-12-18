# Railway Deployment Fix Guide

## Problem
Railway is trying to build from the root directory (which has a package.json for Docusaurus) instead of the backend directory.

## Solution: Set Root Directory in Railway Dashboard

### Step 1: Access Railway Dashboard
1. Open your browser and go to: https://railway.app/dashboard
2. Click on your **book-chatbot-backend** project

### Step 2: Configure Root Directory
1. Click on your **service** (the one you created)
2. Click on **Settings** tab
3. Scroll down to find **Root Directory** or **Source** settings
4. Set **Root Directory** to: `backend`
5. Click **Save** or the setting will auto-save

### Step 3: Verify Configuration Files

Make sure these files exist in your `backend` directory:
- ✅ `railway.json` - Railway configuration
- ✅ `nixpacks.toml` - Build configuration
- ✅ `Procfile` - Start command
- ✅ `runtime.txt` - Python version
- ✅ `requirements.txt` - Python dependencies

### Step 4: Redeploy

After setting the root directory, Railway should automatically redeploy. If not:

**Option A - Dashboard:**
1. Go to **Deployments** tab
2. Click **Redeploy** on the latest deployment

**Option B - CLI:**
In your terminal (where you're logged in):
```bash
cd D:\Book_chatbot\backend
railway up
```

### Step 5: Monitor Deployment

Watch the build logs in the Railway dashboard:
1. Go to **Deployments** tab
2. Click on the active deployment
3. Watch the logs - you should see:
   ```
   Installing Python 3.11...
   Installing dependencies from requirements.txt...
   Starting uvicorn...
   ```

## Alternative: Deploy Backend as Separate Project

If setting root directory doesn't work, create a NEW project just for backend:

### In Your Terminal (logged in to Railway):

```bash
# Navigate to backend directory
cd D:\Book_chatbot\backend

# Unlink from current project (if linked)
# This won't delete your existing project, just unlink this directory
rm -f .railway/railway.toml 2>/dev/null || echo "No existing link"

# Create NEW Railway project from backend directory
railway init

# When prompted:
# - Project name: book-chatbot-backend-python
# - This creates a fresh project linked ONLY to the backend folder

# Set environment variables (IMPORTANT!)
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

# Deploy
railway up

# Get your public URL
railway domain
```

## Expected Output After Successful Deployment

When deployment succeeds, you should see:

```
✓ Build successful
✓ Deployment live

Your service is available at:
https://book-chatbot-backend-production-xxxx.up.railway.app
```

## Test Your Deployment

Once deployed, test these endpoints:

```bash
# Health check
curl https://your-railway-url.railway.app/health

# API docs (open in browser)
https://your-railway-url.railway.app/docs
```

## Update CORS After Deployment

Once you have your Railway URL, update CORS:

```bash
railway variables set CORS_ORIGINS="https://your-railway-url.railway.app,http://localhost:3000,https://your-frontend-url.com"
```

## Troubleshooting

### Still seeing npm build error?
- Make sure Root Directory is set to `backend` in Railway dashboard
- OR create a completely new project from the backend directory

### Python packages not installing?
- Check that `requirements.txt` exists in backend directory
- Verify `nixpacks.toml` has correct Python configuration

### Service won't start?
- Check environment variables are set (especially QWEN_API_KEY, QDRANT_URL)
- View logs in Railway dashboard for specific errors

### CORS errors when testing?
- Update CORS_ORIGINS to include your Railway URL
- Railway will auto-redeploy when you change variables

---

**Follow the steps above and let me know the result!**
