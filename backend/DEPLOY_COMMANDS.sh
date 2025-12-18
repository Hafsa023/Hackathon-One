#!/bin/bash
# Railway Deployment Helper Script
# Run this script to deploy your backend to Railway

echo "========================================"
echo "Railway Deployment Helper"
echo "========================================"
echo ""

# Check if Railway CLI is installed
if ! command -v railway &> /dev/null; then
    echo "ERROR: Railway CLI is not installed!"
    echo "Please install it first: npm install -g @railway/cli"
    exit 1
fi

echo "Railway CLI is installed."
echo ""

# Check authentication
echo "Checking Railway authentication..."
if ! railway whoami &> /dev/null; then
    echo "You are not logged in to Railway."
    echo "Opening login process..."
    railway login
    if [ $? -ne 0 ]; then
        echo "ERROR: Login failed!"
        exit 1
    fi
fi

echo "You are authenticated with Railway."
echo ""

# Check if project is linked
echo "Checking if project is linked..."
if ! railway status &> /dev/null; then
    echo "No Railway project is linked to this directory."
    echo ""
    echo "Choose an option:"
    echo "1. Create a new Railway project"
    echo "2. Link to an existing Railway project"
    echo ""
    read -p "Enter choice (1 or 2): " choice

    if [ "$choice" == "1" ]; then
        echo "Creating new Railway project..."
        railway init
    elif [ "$choice" == "2" ]; then
        echo "Linking to existing project..."
        railway link
    else
        echo "Invalid choice!"
        exit 1
    fi
fi

echo ""
echo "========================================"
echo "Current Project Status:"
echo "========================================"
railway status
echo ""

echo "========================================"
echo "IMPORTANT: Environment Variables"
echo "========================================"
echo ""
echo "Make sure you have set all required environment variables in Railway dashboard:"
echo "- QWEN_API_KEY"
echo "- QWEN_BASE_URL"
echo "- QWEN_CHAT_MODEL"
echo "- QDRANT_URL"
echo "- QDRANT_API_KEY"
echo "- QDRANT_COLLECTION_NAME"
echo "- CORS_ORIGINS"
echo ""
echo "You can set them via Railway dashboard or use:"
echo "  railway variables set KEY=\"value\""
echo ""
read -p "Have you set all environment variables? (y/n): " continue
if [ "$continue" != "y" ] && [ "$continue" != "Y" ]; then
    echo "Please set environment variables first!"
    echo "Opening Railway dashboard..."
    railway open
    exit 0
fi

echo ""
echo "========================================"
echo "Deploying to Railway..."
echo "========================================"
railway up

if [ $? -eq 0 ]; then
    echo ""
    echo "========================================"
    echo "Deployment Successful!"
    echo "========================================"
    echo ""
    echo "Getting your public URL..."
    railway domain
    echo ""
    echo "========================================"
    echo "Next Steps:"
    echo "========================================"
    echo "1. Copy your Railway URL from above"
    echo "2. Update CORS_ORIGINS in Railway dashboard to include your URL"
    echo "3. Update frontend config with your Railway URL"
    echo "4. Test your API at: https://your-url.railway.app/docs"
    echo ""
    echo "To view logs: railway logs"
    echo "To check status: railway status"
    echo ""
else
    echo ""
    echo "========================================"
    echo "Deployment Failed!"
    echo "========================================"
    echo ""
    echo "View logs for details: railway logs"
    echo ""
fi
