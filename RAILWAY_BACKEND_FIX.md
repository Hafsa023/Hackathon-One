# Fix Railway Backend Deployment

## The Problem

Railway is detecting your project as Node.js (because of package.json in root) instead of Python. This causes build failures.

---

## The Solution: Configure Root Directory in Railway

You need to tell Railway to **ONLY** look at the `/backend` folder.

---

## Step-by-Step Fix

### 1. Go to Railway Dashboard
Visit: https://railway.app/dashboard

### 2. Click on Your Service
Click on the backend service (not the project)

### 3. Open Settings
Click the **"Settings"** tab

### 4. Set Root Directory (CRITICAL!)
Scroll down to find **"Root Directory"** or **"Service Settings"**

**Set the value to:**
```
backend
```

**NOT:**
- ❌ `/backend`
- ❌ `./backend`
- ✅ `backend` (correct!)

### 5. Save Changes
Click **"Save"** or **"Update"**

### 6. Redeploy
Railway will automatically trigger a new deployment

---

## What This Does

By setting the root directory to `backend`:
- Railway will use `backend/nixpacks.toml` (Python config)
- Railway will see `backend/requirements.txt` (Python dependencies)
- Railway will ignore the frontend files in root

---

## Verify It's Working

After redeployment:

1. **Check Build Logs**: Should see Python/pip commands, not Node.js
2. **Check Status**: Should say "Deployed" with green checkmark
3. **Test API**: Visit `https://your-backend-url.up.railway.app/docs`

---

## If Still Not Working

### Option A: Create New Service from Backend Folder

1. In Railway, click **"+ New"** → **"GitHub Repo"**
2. Select your repository
3. When prompted for "Root Directory", enter: `backend`
4. Add environment variables
5. Deploy

### Option B: Use Railway CLI

```bash
# Install Railway CLI
npm i -g @railway/cli

# Login
railway login

# Link to your project
railway link

# Set root directory
railway service --root backend

# Deploy
railway up
```

---

## Important: Don't Forget Environment Variables!

After fixing the root directory, make sure you have these variables set:

```bash
QWEN_API_KEY=your-openrouter-key
QDRANT_URL=https://your-cluster.cloud.qdrant.io:6333
QDRANT_API_KEY=your-qdrant-key
DATABASE_URL=postgresql://your-neon-connection-string
CORS_ORIGINS=*
```

See `BACKEND_RAILWAY_SETUP.md` for the complete list.

---

## Alternative: Deploy Backend Only Repository

If Railway keeps having issues, you could:

1. Create a new repository with ONLY the backend folder
2. Connect that repository to Railway
3. Much simpler - no root directory confusion

---

## Need Help?

Check these files:
- `BACKEND_RAILWAY_SETUP.md` - Complete backend setup guide
- `RAILWAY_SETUP_GUIDE.md` - Service setup (OpenRouter, Qdrant, Neon)
- `DEPLOYMENT_CHECKLIST.md` - Step-by-step checklist

---

**The key fix: Set "Root Directory" to `backend` in Railway Settings!**
