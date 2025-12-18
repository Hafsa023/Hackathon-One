@echo off
REM Start script for FastAPI backend on Windows

echo ============================================
echo Starting Physical AI Book Chatbot Backend
echo ============================================

REM Check if virtual environment exists
if not exist "venv\" (
    echo Virtual environment not found. Creating one...
    python -m venv venv
    echo Virtual environment created.
)

REM Activate virtual environment
echo Activating virtual environment...
call venv\Scripts\activate.bat

REM Install/update dependencies
echo Installing dependencies...
pip install -r requirements.txt

REM Check if .env file exists
if not exist ".env" (
    echo .env file not found!
    echo Please copy .env.example to .env and configure it.
    echo Example: copy .env.example .env
    pause
    exit /b 1
)

REM Start the FastAPI server
echo.
echo Starting FastAPI server on http://localhost:8000
echo API Documentation will be available at http://localhost:8000/docs
echo.
uvicorn app.main:app --reload --host 0.0.0.0 --port 8000
