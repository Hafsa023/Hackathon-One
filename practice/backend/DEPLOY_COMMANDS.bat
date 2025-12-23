@echo off
REM Railway Deployment Helper Script
REM Run this script to deploy your backend to Railway

echo ========================================
echo Railway Deployment Helper
echo ========================================
echo.

REM Check if Railway CLI is installed
railway --version >nul 2>&1
if %errorlevel% neq 0 (
    echo ERROR: Railway CLI is not installed!
    echo Please install it first: npm install -g @railway/cli
    pause
    exit /b 1
)

echo Railway CLI is installed.
echo.

REM Check authentication
echo Checking Railway authentication...
railway whoami >nul 2>&1
if %errorlevel% neq 0 (
    echo You are not logged in to Railway.
    echo Opening login process...
    railway login
    if %errorlevel% neq 0 (
        echo ERROR: Login failed!
        pause
        exit /b 1
    )
)

echo You are authenticated with Railway.
echo.

REM Check if project is linked
echo Checking if project is linked...
railway status >nul 2>&1
if %errorlevel% neq 0 (
    echo No Railway project is linked to this directory.
    echo.
    echo Choose an option:
    echo 1. Create a new Railway project
    echo 2. Link to an existing Railway project
    echo.
    set /p choice="Enter choice (1 or 2): "

    if "%choice%"=="1" (
        echo Creating new Railway project...
        railway init
    ) else if "%choice%"=="2" (
        echo Linking to existing project...
        railway link
    ) else (
        echo Invalid choice!
        pause
        exit /b 1
    )
)

echo.
echo ========================================
echo Current Project Status:
echo ========================================
railway status
echo.

echo ========================================
echo IMPORTANT: Environment Variables
echo ========================================
echo.
echo Make sure you have set all required environment variables in Railway dashboard:
echo - QWEN_API_KEY
echo - QWEN_BASE_URL
echo - QWEN_CHAT_MODEL
echo - QDRANT_URL
echo - QDRANT_API_KEY
echo - QDRANT_COLLECTION_NAME
echo - CORS_ORIGINS
echo.
echo You can set them via Railway dashboard or use:
echo   railway variables set KEY="value"
echo.
set /p continue="Have you set all environment variables? (y/n): "
if /i not "%continue%"=="y" (
    echo Please set environment variables first!
    echo Opening Railway dashboard...
    railway open
    pause
    exit /b 0
)

echo.
echo ========================================
echo Deploying to Railway...
echo ========================================
railway up

if %errorlevel% equ 0 (
    echo.
    echo ========================================
    echo Deployment Successful!
    echo ========================================
    echo.
    echo Getting your public URL...
    railway domain
    echo.
    echo ========================================
    echo Next Steps:
    echo ========================================
    echo 1. Copy your Railway URL from above
    echo 2. Update CORS_ORIGINS in Railway dashboard to include your URL
    echo 3. Update frontend config with your Railway URL
    echo 4. Test your API at: https://your-url.railway.app/docs
    echo.
    echo To view logs: railway logs
    echo To check status: railway status
    echo.
) else (
    echo.
    echo ========================================
    echo Deployment Failed!
    echo ========================================
    echo.
    echo View logs for details: railway logs
    echo.
)

pause
