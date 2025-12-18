@echo off
REM Quick start script - Run this to start the Docusaurus frontend

echo ============================================
echo Starting Physical AI Book Frontend
echo ============================================

REM Check if node_modules exists
if not exist "node_modules\" (
    echo Installing dependencies...
    npm install
)

REM Start Docusaurus
echo.
echo Starting Docusaurus on http://localhost:3000
echo.
npm start
