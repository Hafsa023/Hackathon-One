# Deploy Backend to Render.com (Much Easier!)

Railway is having issues with your setup. Render.com is MUCH simpler for Python backends.

---

## Why Render?

- ✅ Better free tier (750 hours/month)
- ✅ Automatically detects Python projects correctly
- ✅ Simpler configuration
- ✅ No root directory confusion

---

## Step-by-Step: Deploy to Render

### Step 1: Sign Up

1. Go to: https://render.com/
2. Click **"Get Started"**
3. Sign up with **GitHub** (easiest)

### Step 2: Create New Web Service

1. After login, click **"+ New"** → **"Web Service"**
2. Click **"Connect GitHub"** and authorize Render
3. Find and select your repository: **Physical-AI-Chatbot**
4. Click **"Connect"**

### Step 3: Configure Service

Fill in these settings:

**Name**: `physical-ai-backend` (or any name you want)

**Region**: Choose closest to you

**Branch**: `main`

**Root Directory**: `backend` ✅ (This actually works on Render!)

**Runtime**: `Python 3` (should auto-detect)

**Build Command**:
```
pip install -r requirements.txt
```

**Start Command**:
```
uvicorn app.main:app --host 0.0.0.0 --port $PORT
```

**Plan**: **Free** ✅

### Step 4: Add Environment Variables

Click **"Advanced"** → Scroll to **"Environment Variables"**

Add these (click **"Add Environment Variable"** for each):

```
QWEN_API_KEY=your-openrouter-key-here
QWEN_BASE_URL=https://openrouter.ai/api/v1
QWEN_CHAT_MODEL=qwen/qwen-2.5-72b-instruct
QDRANT_URL=https://your-cluster.cloud.qdrant.io:6333
QDRANT_API_KEY=your-qdrant-key-here
QDRANT_COLLECTION_NAME=book_chatbot
DATABASE_URL=postgresql://your-neon-connection-string
CORS_ORIGINS=*
```

### Step 5: Deploy!

1. Click **"Create Web Service"** at the bottom
2. Render will start building and deploying
3. Wait 3-5 minutes

---

## After Deployment

### Get Your Backend URL:

1. Go to your service dashboard
2. You'll see a URL like: `https://physical-ai-backend.onrender.com`
3. Copy this URL

### Test It:

Visit: `https://your-backend.onrender.com/docs`

Should see FastAPI documentation! ✅

---

## Update Your Vercel Frontend

1. Go to Vercel dashboard
2. Your project → Settings → Environment Variables
3. Update `REACT_APP_API_URL` to your Render URL:
   ```
   REACT_APP_API_URL=https://your-backend.onrender.com
   ```
4. Redeploy Vercel

---

## Important Notes

### Free Tier Limitations:

- Service spins down after 15 minutes of inactivity
- First request after sleep takes ~30 seconds to wake up
- 750 hours/month free (enough for 1 service running 24/7)

### Keep It Awake (Optional):

Use a free service like UptimeRobot to ping your backend every 14 minutes:
- https://uptimerobot.com/
- Add monitor with your Render URL
- Keeps it from sleeping

---

## Troubleshooting

### Build fails?
- Check logs in Render dashboard
- Verify environment variables are set
- Check root directory is set to `backend`

### Can't connect from frontend?
- Update CORS_ORIGINS in Render with your Vercel URL
- Check REACT_APP_API_URL in Vercel is correct
- Test backend /docs endpoint first

### Service won't start?
- Check Start Command is correct
- Verify all required env variables are set
- Look at logs for error messages

---

## Comparison: Render vs Railway

| Feature | Render | Railway |
|---------|--------|---------|
| Python Detection | ✅ Perfect | ❌ Broken for monorepos |
| Root Directory | ✅ Works | ❌ Doesn't work |
| Free Tier | ✅ 750 hrs | ⚠️ $5 credit |
| Setup Difficulty | ✅ Easy | ❌ Complex |

**Verdict**: Use Render for this project!

---

## Summary

1. Sign up at render.com
2. Connect GitHub repo
3. Set root directory to `backend`
4. Add environment variables
5. Deploy!
6. Update Vercel with new backend URL

**Total time: 10 minutes** ⏱️

Much easier than fighting with Railway! 🎉

---

## Need Environment Variables?

See these files:
- `RAILWAY_SETUP_GUIDE.md` - How to get API keys
- `DEPLOYMENT_CHECKLIST.md` - Complete setup checklist

The same API keys work for Render!

---

Good luck! 🚀
