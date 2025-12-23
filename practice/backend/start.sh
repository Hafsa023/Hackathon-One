#!/bin/bash
# Start script for FastAPI backend on Linux/Mac

echo "============================================"
echo "Starting Physical AI Book Chatbot Backend"
echo "============================================"

# Check if virtual environment exists
if [ ! -d "venv" ]; then
    echo "Virtual environment not found. Creating one..."
    python3 -m venv venv
    echo "Virtual environment created."
fi

# Activate virtual environment
echo "Activating virtual environment..."
source venv/bin/activate

# Install/update dependencies
echo "Installing dependencies..."
pip install -r requirements.txt

# Check if .env file exists
if [ ! -f ".env" ]; then
    echo ".env file not found!"
    echo "Please copy .env.example to .env and configure it."
    echo "Example: cp .env.example .env"
    exit 1
fi

# Start the FastAPI server
echo ""
echo "Starting FastAPI server on http://localhost:8000"
echo "API Documentation will be available at http://localhost:8000/docs"
echo ""
uvicorn app.main:app --reload --host 0.0.0.0 --port 8000
