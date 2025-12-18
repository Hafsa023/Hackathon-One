# Railway Deployment Guide

## Prerequisites Checklist

- [x] Railway CLI installed (v4.15.0)
- [ ] Railway account created at https://railway.app
- [ ] Backend running locally and tested
- [ ] All API keys and credentials ready

## Step 1: Authenticate with Railway

Open a new terminal and run:

```bash
cd D:\Book_chatbot\backend
railway login
```

This will open your browser. Complete the authentication process.

## Step 2: Verify Authentication

After logging in, verify your account:

```bash
railway whoami
```

You should see your Railway account information.

## Step 3: Initialize Railway Project

Choose one option:

### Option A: Create New Project (First Time)

```bash
railway init
```

- Project name: `book-chatbot-backend`
- This will create a new Railway project and link your local directory

### Option B: Link Existing Project

If you already created a project on Railway dashboard:

```bash
railway list
railway link
```

Select your project from the list.

## Step 4: Configure Environment Variables

You have two options to set environment variables:

### Option A: Using Railway Dashboard (Recommended)

1. Go to https://railway.app/dashboard
2. Click on your `book-chatbot-backend` project
3. Click on the service (or create one if needed)
4. Go to "Variables" tab
5. Add these variables one by one:

```
QWEN_API_KEY=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
QWEN_BASE_URL=https://openrouter.ai/api/v1
QWEN_CHAT_MODEL=qwen/qwen-2.5-72b-instruct
QDRANT_URL=https://38e65eab-cb19-47f2-a63a-c792711d36fe.europe-west3-0.gcp.cloud.qdrant.io:6333
QDRANT_API_KEY=eyJhbGciOiJIUzI1NiIsInR5cCI6IkpXVCJ9.eyJhY2Nlc3MiOiJtIn0.rUAwVZWmdTCS4knmEoiUyZx9xSY2p3DndkjtL97Yaj0
QDRANT_COLLECTION_NAME=book_chatbot
DATABASE_URL=postgresql://neondb_owner:YOUR_PASSWORD@ep-xxx.us-east-2.aws.neon.tech/neondb?sslmode=require
CHUNK_SIZE=400
CHUNK_OVERLAP=50
TOP_K_RESULTS=5
MAX_CONTEXT_LENGTH=4000
CORS_ORIGINS=http://localhost:3000,https://your-production-url.com
```

**IMPORTANT**: Update `CORS_ORIGINS` to include your Railway backend URL and frontend URL after deployment.

### Option B: Using Railway CLI

Run these commands one by one:

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
```

## Step 5: Configure Start Command

Railway needs to know how to start your application.

### Option A: Using Railway Dashboard

1. Go to your project settings
2. Find "Deploy" section
3. Set start command to:
   ```
   uvicorn app.main:app --host 0.0.0.0 --port $PORT
   ```

### Option B: Create railway.json

Create a `railway.json` file in the backend directory (this will be done automatically).

## Step 6: Deploy to Railway

From the backend directory:

```bash
railway up
```

This will:
- Upload your code
- Install dependencies from requirements.txt
- Start your FastAPI application

## Step 7: Get Your Public URL

After deployment completes:

```bash
railway domain
```

If no domain is assigned yet, generate one:

```bash
railway domain generate
```

Or add a custom domain via the Railway dashboard.

## Step 8: Update CORS Settings

Once you have your Railway URL (e.g., `https://book-chatbot-backend-production.up.railway.app`):

1. Go to Railway dashboard → Variables
2. Update `CORS_ORIGINS` to include:
   ```
   https://book-chatbot-backend-production.up.railway.app,https://your-frontend-url.com,http://localhost:3000
   ```
3. Railway will automatically redeploy

## Step 9: Verify Deployment

Test your deployed API:

```bash
# Check health endpoint
curl https://your-railway-url.railway.app/health

# Check docs
# Open in browser: https://your-railway-url.railway.app/docs
```

## Step 10: Update Frontend Configuration

Update `src/config/chatConfig.ts` with your Railway backend URL:

```typescript
export const API_BASE_URL = 'https://your-railway-url.railway.app';
```

## Troubleshooting

### Check Deployment Logs

```bash
railway logs
```

### Check Service Status

```bash
railway status
```

### Redeploy

If you need to redeploy:

```bash
railway up --detach
```

### Common Issues

1. **Port Error**: Make sure your start command uses `$PORT` variable
2. **Module Not Found**: Check that `requirements.txt` is complete
3. **CORS Error**: Update `CORS_ORIGINS` to include your Railway URL
4. **Database Connection**: Verify `DATABASE_URL` is correct (optional for basic chatbot)

## Next Steps After Deployment

- [ ] Verify `/health` endpoint responds
- [ ] Test `/docs` API documentation
- [ ] Run document ingestion: `POST /ingest`
- [ ] Test chat endpoint: `POST /chat`
- [ ] Update frontend with production URL
- [ ] Deploy frontend to Vercel/Netlify
- [ ] Update CORS to include frontend URL

## Useful Railway Commands

```bash
# View all projects
railway list

# Switch between projects
railway link

# View environment variables
railway variables

# View logs in real-time
railway logs --follow

# Open project in browser
railway open

# Check current status
railway status
```

## Cost Estimation

Railway offers:
- **Free Tier**: $5 credit per month (enough for testing)
- **Pro Plan**: $20/month with $20 credit included

Your backend should use minimal resources for a chatbot application.

---

**Ready to deploy? Follow the steps above in order!**
