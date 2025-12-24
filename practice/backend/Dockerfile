# Physical AI Book Chatbot Backend
# Simple single-stage build for Railway

FROM python:3.11-slim

WORKDIR /app

# Install build dependencies
RUN apt-get update && apt-get install -y --no-install-recommends \
    build-essential \
    && rm -rf /var/lib/apt/lists/*

# Install Python dependencies
COPY requirements.txt .
RUN pip install --no-cache-dir -r requirements.txt

# Copy application code
COPY app/ app/
COPY scripts/ scripts/

# Expose port
EXPOSE 8000

# Note: Start command is set in Railway dashboard or Procfile
