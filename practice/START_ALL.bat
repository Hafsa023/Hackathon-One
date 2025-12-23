@echo off
REM Start both frontend and backend in separate windows

echo ============================================
echo Starting Full Application Stack
echo ============================================
echo.
echo This will open two windows:
echo 1. Backend (FastAPI on port 8000)
echo 2. Frontend (Docusaurus on port 3000)
echo.
echo Press any key to continue...
pause > nul

REM Start backend in new window
start "Physical AI Backend" cmd /k "cd /d %~dp0 && START_BACKEND.bat"

REM Wait a bit for backend to initialize
echo Waiting for backend to initialize...
timeout /t 5 /nobreak > nul

REM Start frontend in new window
start "Physical AI Frontend" cmd /k "cd /d %~dp0 && START_FRONTEND.bat"

echo.
echo Both services are starting...
echo Backend: http://localhost:8000/docs
echo Frontend: http://localhost:3000
echo.
