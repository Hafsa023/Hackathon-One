# Railway Deployment Checklist

Complete these steps in order. Check off each one as you go.

## ☐ Step 1: Get OpenRouter API Key (2 minutes)

1. ☐ Go to https://openrouter.ai/
2. ☐ Click "Sign In" and create account
3. ☐ Go to https://openrouter.ai/keys
4. ☐ Click "Create Key"
5. ☐ Copy the key (starts with `sk-or-v1-...`)
6. ☐ Save it somewhere safe

**You should have:**
```
QWEN_API_KEY=sk-or-v1-xxxxxxxxxxxxxxxxxxxxxxxx
```

---

## ☐ Step 2: Create Qdrant Vector Database (5 minutes)

1. ☐ Go to https://cloud.qdrant.io/
2. ☐ Sign up with Google/GitHub
3. ☐ Click "Create Cluster"
4. ☐ Choose "Free tier"
5. ☐ Name it: `physical-ai-book`
6. ☐ Wait 2 minutes for it to provision
7. ☐ Click on cluster to see details
8. ☐ Copy the Cluster URL
9. ☐ Click "Show" to reveal API Key
10. ☐ Copy the API Key

**You should have:**
```
QDRANT_URL=https://xxxxxxxx.aws.cloud.qdrant.io:6333
QDRANT_API_KEY=xxxxxxxxxxxxxxxxxxxxxxxx
```

---

## ☐ Step 3: Create Neon PostgreSQL Database (3 minutes)

1. ☐ Go to https://neon.tech/
2. ☐ Click "Sign Up" with Google/GitHub
3. ☐ Create new project named: `physical-ai-book`
4. ☐ Database name: `chatbot`
5. ☐ Choose region closest to you
6. ☐ Click "Create Project"
7. ☐ Find "Connection Details" section
8. ☐ Copy the connection string (postgresql://...)
9. ☐ **IMPORTANT**: Save the password shown - you won't see it again!

**You should have:**
```
DATABASE_URL=postgresql://user:password@ep-xxx.region.aws.neon.tech/chatbot?sslmode=require
```

---

## ☐ Step 4: Add Variables to Railway (5 minutes)

1. ☐ Go to https://railway.app/
2. ☐ Open your project
3. ☐ Click on your **backend service**
4. ☐ Click **"Variables"** tab
5. ☐ Add these variables one by one:

### Required Variables (copy your actual values):

```bash
QWEN_API_KEY=<paste your OpenRouter key here>
QWEN_BASE_URL=https://openrouter.ai/api/v1
QWEN_CHAT_MODEL=qwen/qwen-2.5-72b-instruct

QDRANT_URL=<paste your Qdrant URL here>
QDRANT_API_KEY=<paste your Qdrant API key here>
QDRANT_COLLECTION_NAME=book_chatbot

DATABASE_URL=<paste your Neon connection string here>

CORS_ORIGINS=*
```

6. ☐ Click "Add" after each variable
7. ☐ Wait for Railway to automatically redeploy (2-3 minutes)

---

## ☐ Step 5: Verify Backend Deployment

1. ☐ In Railway, go to your backend service
2. ☐ Click "Settings" tab
3. ☐ Scroll to "Networking" section
4. ☐ Click "Generate Domain" (if not already generated)
5. ☐ Copy the domain URL
6. ☐ Open in browser: `https://your-backend-domain.up.railway.app/docs`
7. ☐ You should see FastAPI documentation page!

**Backend URL:**
```
https://_____________________________.up.railway.app
```

---

## ☐ Step 6: Update Frontend (if needed)

If you have a separate frontend service:

1. ☐ In Railway, go to your **frontend service**
2. ☐ Click "Variables" tab
3. ☐ Add variable:
```bash
REACT_APP_API_URL=https://your-backend-domain.up.railway.app
```
4. ☐ Wait for redeployment

---

## ✅ All Done!

Your chatbot should now be deployed and working!

### Test it:
- ☐ Backend API: `https://your-backend.up.railway.app/docs`
- ☐ Frontend: `https://your-frontend.up.railway.app`

### If something fails:
1. Check Railway logs (Logs tab in your service)
2. Verify all environment variables are correct
3. Make sure you copied the complete connection strings
4. Read RAILWAY_SETUP_GUIDE.md for detailed troubleshooting

---

## Quick Reference Links

- OpenRouter Dashboard: https://openrouter.ai/activity
- Qdrant Dashboard: https://cloud.qdrant.io/
- Neon Dashboard: https://console.neon.tech/
- Railway Dashboard: https://railway.app/dashboard

---

**Estimated time: 15-20 minutes total**

Good luck! 🚀
