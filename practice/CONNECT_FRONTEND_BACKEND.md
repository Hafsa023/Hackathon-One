# Connect Your Frontend (Vercel) to Backend (Railway)

Your backend is deployed! Now let's connect everything together.

---

## Step 1: Get Your Railway Backend URL ✅

1. Go to Railway dashboard: https://railway.app/dashboard
2. Click on your backend service
3. Go to **"Settings"** tab
4. Find **"Networking"** → **"Public Domain"**
5. Copy the URL (e.g., `https://xyz123.up.railway.app`)

**Your Backend URL:**
```
https://_____________________________.up.railway.app
```

---

## Step 2: Test Your Backend 🧪

Open in browser:
```
https://your-backend-url.up.railway.app/docs
```

**What you should see:**
- FastAPI Swagger documentation page
- List of all API endpoints (health, chat, auth, etc.)
- Interactive API testing interface

**Test the health endpoint:**
```
https://your-backend-url.up.railway.app/health
```

Should return:
```json
{"status": "healthy"}
```

---

## Step 3: Update Vercel Environment Variable 🔧

### Option A: Via Vercel Dashboard (Recommended)

1. Go to your Vercel project: https://vercel.com/dashboard
2. Click on your project
3. Go to **"Settings"** → **"Environment Variables"**
4. Click **"Add New"**
5. Add:
   - **Name**: `REACT_APP_API_URL`
   - **Value**: `https://your-backend-url.up.railway.app` (no trailing slash!)
   - **Environment**: Check all (Production, Preview, Development)
6. Click **"Save"**
7. **Redeploy**: Go to "Deployments" tab → Click "..." on latest → "Redeploy"

### Option B: Via Code (Alternative)

Update `src/config/chatConfig.ts`:
```typescript
API_BASE_URL: 'https://your-actual-backend-url.up.railway.app',
```

Then commit and push to trigger Vercel redeploy.

---

## Step 4: Update CORS in Railway Backend 🔐

Your backend needs to allow requests from your Vercel frontend:

1. In Railway dashboard, go to your backend service
2. Go to **"Variables"** tab
3. Find or add `CORS_ORIGINS`
4. Set value to:
   ```
   https://your-vercel-app.vercel.app
   ```

   Or to allow all (testing only):
   ```
   *
   ```

5. Railway will auto-redeploy

**Important**: Replace `your-vercel-app.vercel.app` with your actual Vercel URL!

---

## Step 5: Test Full Integration 🎉

1. **Visit your Vercel frontend**:
   ```
   https://your-vercel-app.vercel.app
   ```

2. **Open the chat widget** (bottom-right corner)

3. **Ask a test question**:
   - "What is embodied intelligence?"
   - "How do I set up ROS 2?"

4. **Check for responses**:
   - If working: You'll get AI responses ✅
   - If error: Check browser console (F12) for errors

---

## Troubleshooting 🔍

### Error: "Unable to connect to server"

**Check:**
1. Backend URL is correct in Vercel environment variable
2. Backend is running (visit `/docs` endpoint)
3. CORS is configured with your Vercel URL
4. No typos in URLs (no trailing slashes!)

### Error: "CORS policy blocked"

**Fix:**
1. Go to Railway → Variables
2. Update `CORS_ORIGINS` with your Vercel URL
3. Wait for redeploy
4. Clear browser cache and try again

### Error: "404 Not Found"

**Check:**
1. Backend URL is correct
2. API endpoints exist (check `/docs`)
3. Backend deployed from `/backend` directory

### Chat widget not appearing

**Check:**
1. Vercel build succeeded
2. Check browser console for JavaScript errors
3. Try hard refresh (Ctrl+Shift+R or Cmd+Shift+R)

---

## Verify Everything Works ✅

### Backend Health Check:
```
https://your-backend.up.railway.app/health
```
Should return: `{"status": "healthy"}`

### Backend API Docs:
```
https://your-backend.up.railway.app/docs
```
Should show FastAPI documentation

### Frontend:
```
https://your-vercel-app.vercel.app
```
Should load with chat widget visible

### Full Test:
1. Open frontend
2. Click chat widget
3. Ask: "What is embodied intelligence?"
4. Get AI response ✅

---

## Environment Variables Summary

### Railway (Backend):
```bash
QWEN_API_KEY=sk-or-v1-xxx...
QDRANT_URL=https://xxx.cloud.qdrant.io:6333
QDRANT_API_KEY=xxx...
DATABASE_URL=postgresql://xxx...
CORS_ORIGINS=https://your-vercel-app.vercel.app
```

### Vercel (Frontend):
```bash
REACT_APP_API_URL=https://your-backend.up.railway.app
```

---

## Next Steps After Connection 🚀

1. **Populate Vector Database** with your book content
2. **Test all chatbot features**
3. **Monitor logs** in Railway and Vercel
4. **Set up monitoring/alerts** (optional)
5. **Configure custom domain** (optional)

---

## Quick Links 🔗

- Railway Dashboard: https://railway.app/dashboard
- Vercel Dashboard: https://vercel.com/dashboard
- OpenRouter Usage: https://openrouter.ai/activity
- Qdrant Dashboard: https://cloud.qdrant.io/
- Neon Dashboard: https://console.neon.tech/

---

**Need help?** Check the logs:
- Railway: Service → "Logs" tab
- Vercel: Project → "Deployments" → Click deployment → "Logs"
- Browser: Press F12 → "Console" tab

Good luck! 🎉
