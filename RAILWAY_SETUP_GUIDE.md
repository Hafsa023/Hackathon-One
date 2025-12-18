# Complete Railway Deployment Setup Guide

This guide will walk you through setting up all required services and deploying your Physical AI Book chatbot to Railway.

## Overview

Your application requires three external services:
1. **OpenRouter** - For AI chat functionality (Qwen model)
2. **Qdrant Cloud** - For vector search/embeddings
3. **Neon** - For PostgreSQL database

All three offer generous free tiers!

---

## Step 1: OpenRouter API Key (AI Chat)

### Sign Up & Get API Key:

1. Go to **https://openrouter.ai/**
2. Click "Sign In" (top right)
3. Sign up with Google/GitHub or email
4. After login, go to **https://openrouter.ai/keys**
5. Click "Create Key"
6. Name it: `Physical-AI-Book`
7. Copy the API key (starts with `sk-or-v1-...`)

**Cost**: Pay-as-you-go. Free $1 credit to start. Very cheap (~$0.50 per 1M tokens for Qwen)

**Save this for later:**
```
QWEN_API_KEY=sk-or-v1-xxxxxxxxxxxxxxxxxxxxxxxx
```

---

## Step 2: Qdrant Cloud (Vector Database)

### Sign Up & Create Cluster:

1. Go to **https://cloud.qdrant.io/**
2. Click "Get Started Free"
3. Sign up with Google/GitHub
4. After login, click "Create Cluster"
5. Choose:
   - **Plan**: Free tier (1GB, perfect for this project)
   - **Region**: Choose closest to you
   - **Cluster Name**: `physical-ai-book`
6. Click "Create"

### Get Connection Details:

1. Wait ~2 minutes for cluster to provision
2. Click on your cluster name
3. You'll see:
   - **Cluster URL**: `https://xxxxxxxx-xxxx-xxxx-xxxx-xxxxxxxxxxxx.us-east-1-0.aws.cloud.qdrant.io:6333`
   - **API Key**: Click "Show" to reveal

**Save these for later:**
```
QDRANT_URL=https://your-cluster-id.region.aws.cloud.qdrant.io:6333
QDRANT_API_KEY=your-api-key-here
```

---

## Step 3: Neon PostgreSQL Database

### Sign Up & Create Database:

1. Go to **https://neon.tech/**
2. Click "Sign Up" (top right)
3. Sign up with Google/GitHub
4. After login, you'll be prompted to create your first project
5. Configure:
   - **Project Name**: `physical-ai-book`
   - **Database Name**: `chatbot`
   - **Region**: Choose closest to you
6. Click "Create Project"

### Get Connection String:

1. On your project dashboard, look for "Connection Details"
2. Select:
   - **Database**: `chatbot`
   - **Role**: `owner`
   - **Connection String**: Toggle to show
3. Copy the connection string (looks like: `postgresql://user:password@ep-xxx.region.aws.neon.tech/chatbot?sslmode=require`)

**Important**: The password is shown only once! Save it.

**Save this for later:**
```
DATABASE_URL=postgresql://user:password@ep-xxx.region.aws.neon.tech/chatbot?sslmode=require
```

---

## Step 4: Configure Railway Environment Variables

Now that you have all the credentials, let's add them to Railway:

### Add Variables to Railway:

1. Go to **https://railway.app/**
2. Open your project
3. Click on your **backend service** (not frontend)
4. Click the **"Variables"** tab
5. Click **"+ New Variable"**
6. Add each of these variables:

```bash
# Required - AI Chat
QWEN_API_KEY=sk-or-v1-your-actual-key-here
QWEN_BASE_URL=https://openrouter.ai/api/v1
QWEN_CHAT_MODEL=qwen/qwen-2.5-72b-instruct

# Required - Vector Database
QDRANT_URL=https://your-actual-cluster-url.aws.cloud.qdrant.io:6333
QDRANT_API_KEY=your-actual-qdrant-key-here
QDRANT_COLLECTION_NAME=book_chatbot

# Required - PostgreSQL
DATABASE_URL=postgresql://your-actual-connection-string-here

# Optional - Configuration
CHUNK_SIZE=400
CHUNK_OVERLAP=50
TOP_K_RESULTS=5
MAX_CONTEXT_LENGTH=4000

# Optional - CORS (allows all origins)
CORS_ORIGINS=*
```

### Tips:
- **Copy-paste each variable name and value carefully**
- **Don't include quotes** around the values in Railway
- **Replace** `your-actual-key-here` with your real credentials
- Click **"Add"** after each variable

---

## Step 5: Deploy!

Once you've added all variables:

1. Railway will **automatically redeploy** your backend
2. Wait 2-3 minutes for the build to complete
3. Check the **"Deployments"** tab to see progress
4. Look for "Deployment successful" ✅

### Verify Deployment:

1. Click on your backend service
2. Go to **"Settings"** tab
3. Find **"Public Domain"** section
4. Click **"Generate Domain"**
5. Copy the URL (e.g., `https://your-app.up.railway.app`)
6. Test it by visiting: `https://your-app.up.railway.app/docs`
7. You should see the FastAPI documentation page!

---

## Step 6: Update Frontend to Use Backend

You need to update your frontend to point to the Railway backend URL:

1. In your Railway project, click on your **frontend service**
2. Go to **"Variables"** tab
3. Add this variable:

```bash
REACT_APP_API_URL=https://your-backend-domain.up.railway.app
```

Replace `your-backend-domain` with the actual domain from Step 5.

---

## Troubleshooting

### Backend won't start?
- Check all environment variables are set correctly
- Look at the **"Logs"** tab for error messages
- Verify your API keys are valid

### Frontend can't connect to backend?
- Make sure `CORS_ORIGINS=*` is set in backend variables
- Check the backend domain is correct in frontend variables
- Test backend API at `/docs` endpoint

### Qdrant connection errors?
- Verify your cluster is running in Qdrant dashboard
- Check URL includes `:6333` port
- Make sure API key is correct

### Database connection errors?
- Verify Neon project is active
- Check connection string includes `?sslmode=require`
- Test connection in Neon dashboard

---

## Next Steps

After successful deployment:

1. **Populate the vector database** with your book content
2. **Test the chatbot** functionality
3. **Monitor usage** in OpenRouter, Qdrant, and Neon dashboards
4. **Set up custom domain** (optional)

---

## Cost Estimates (Free Tier Limits)

- **OpenRouter**: $1 free credit, then ~$0.50 per 1M tokens
- **Qdrant Cloud**: 1GB free forever (enough for ~100k documents)
- **Neon**: 512MB storage + 3GB transfer free forever
- **Railway**: $5 free credit/month (usually enough for hobby projects)

**Total monthly cost for low-medium usage**: $0-5

---

## Need Help?

If you encounter issues:
1. Check the logs in Railway dashboard
2. Verify all environment variables are set
3. Test each service independently
4. Create an issue in the GitHub repository

Good luck with your deployment! 🚀
