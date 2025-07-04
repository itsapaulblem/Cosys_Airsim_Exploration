@echo off
setlocal enabledelayedexpansion

:: AirSim Container Management Script for Windows
:: This script manages the containerized AirSim setup

:: Colors for output (PowerShell)
set "GREEN=[32m"
set "RED=[31m"
set "YELLOW=[33m"
set "BLUE=[34m"
set "NC=[0m"

:: Default values
set "PROFILE=full-stack"
set "DISPLAY_MODE=vnc"
set "DETACHED=-d"
set "CLEANUP=false"

:: Script location
set "SCRIPT_DIR=%~dp0"
set "DOCKER_DIR=%SCRIPT_DIR%"

:parse_args
if "%~1"=="" goto main_execution
if "%~1"=="--profile" (
    set "PROFILE=%~2"
    shift
    shift
    goto parse_args
)
if "%~1"=="--display" (
    set "DISPLAY_MODE=%~2"
    shift
    shift
    goto parse_args
)
if "%~1"=="--foreground" (
    set "DETACHED="
    shift
    goto parse_args
)
if "%~1"=="--cleanup" (
    set "CLEANUP=true"
    shift
    goto parse_args
)
if "%~1"=="--help" goto show_help
if "%~1"=="build" (
    set "COMMAND=build"
    shift
    goto parse_args
)
if "%~1"=="run" (
    set "COMMAND=run"
    shift
    goto parse_args
)
if "%~1"=="stop" (
    set "COMMAND=stop"
    shift
    goto parse_args
)
if "%~1"=="clean" (
    set "COMMAND=clean"
    shift
    goto parse_args
)
if "%~1"=="status" (
    set "COMMAND=status"
    shift
    goto parse_args
)
if "%~1"=="test" (
    set "COMMAND=test"
    shift
    goto parse_args
)
if "%~1"=="shell" (
    set "COMMAND=shell"
    shift
    goto parse_args
)
if "%~1"=="logs" (
    set "COMMAND=logs"
    shift
    goto parse_args
)
echo Unknown option: %~1
goto show_help

:show_help
echo.
echo AirSim Container Management Script
echo.
echo Usage: %~nx0 [OPTIONS] [COMMAND]
echo.
echo Commands:
echo   build           Build the AirSim container images
echo   run             Run the full stack
echo   stop            Stop all containers
echo   clean           Remove all containers and images
echo   status          Show container status
echo   test            Run connectivity tests
echo   shell           Open shell in AirSim container
echo   logs            Show container logs
echo.
echo Options:
echo   --profile PROFILE   Docker Compose profile (default: full-stack)
echo   --display MODE      Display mode (default: vnc)
echo   --foreground        Run in foreground (default: background)
echo   --cleanup           Remove existing containers before running
echo   --help              Show this help message
echo.
echo Examples:
echo   %~nx0 build                    # Build all images
echo   %~nx0 run                      # Run full stack with VNC
echo   %~nx0 run --display headless   # Run headless mode
echo   %~nx0 status                   # Show container status
echo.
echo VNC Access:
echo   URL: http://localhost:5900
echo.
echo API Access:
echo   AirSim API: localhost:41451
echo   Python: airsim.MultirotorClient("localhost")
echo.
goto :eof

:check_prerequisites
echo === Checking Prerequisites ===
where docker >nul 2>&1
if errorlevel 1 (
    echo ERROR: Docker is not installed or not in PATH
    exit /b 1
)

docker compose version >nul 2>&1
if errorlevel 1 (
    echo ERROR: Docker Compose is not available
    exit /b 1
)

docker info >nul 2>&1
if errorlevel 1 (
    echo ERROR: Docker daemon is not running
    exit /b 1
)

echo INFO: Prerequisites check passed
goto :eof

:build_images
echo === Building AirSim Container Images ===
cd /d "%DOCKER_DIR%"
docker compose build
if errorlevel 1 (
    echo ERROR: Build failed
    exit /b 1
)
echo INFO: Build completed successfully
goto :eof

:run_stack
echo === Running AirSim Stack ===
cd /d "%DOCKER_DIR%"

if "%CLEANUP%"=="true" (
    echo WARNING: Cleaning up existing containers...
    docker compose --profile %PROFILE% down --remove-orphans >nul 2>&1
)

:: Set environment variables
set "DISPLAY_MODE=%DISPLAY_MODE%"

echo INFO: Starting stack with profile: %PROFILE%
echo INFO: Display mode: %DISPLAY_MODE%

if "%DETACHED%"=="-d" (
    docker compose --profile %PROFILE% up %DETACHED%
    echo INFO: Stack started in background
    echo INFO: Access VNC at: http://localhost:5900
    echo INFO: AirSim API at: localhost:41451
    timeout /t 3 >nul
    call :show_status
) else (
    echo INFO: Starting in foreground mode (Ctrl+C to stop)
    docker compose --profile %PROFILE% up
)
goto :eof

:stop_stack
echo === Stopping AirSim Stack ===
cd /d "%DOCKER_DIR%"
docker compose down
echo INFO: Stack stopped
goto :eof

:clean_all
echo === Cleaning Up All Resources ===
cd /d "%DOCKER_DIR%"
echo WARNING: This will remove all containers, networks, and images
set /p "REPLY=Are you sure? (y/N): "
if /i "%REPLY%"=="y" (
    docker compose down --volumes --remove-orphans
    docker rmi airsim:latest px4-airsim:latest >nul 2>&1
    docker image prune -f >nul 2>&1
    echo INFO: Cleanup completed
) else (
    echo INFO: Cleanup cancelled
)
goto :eof

:show_status
echo === Container Status ===
cd /d "%DOCKER_DIR%"
docker compose ps
echo.
echo === Network Information ===
docker network inspect airsim-network --format "{{range .Containers}}{{.Name}}: {{.IPv4Address}}{{end}}" 2>nul || echo Network not found
echo.
echo === Port Mappings ===
for /f "tokens=*" %%i in ('docker compose port airsim 41451 2^>nul') do echo AirSim API: %%i
for /f "tokens=*" %%i in ('docker compose port airsim 5900 2^>nul') do echo VNC Access: %%i
goto :eof

:run_tests
echo === Running Connectivity Tests ===
cd /d "%DOCKER_DIR%"

docker ps | findstr airsim-container >nul
if errorlevel 1 (
    echo INFO: Starting test environment...
    set "DISPLAY_MODE=headless"
    set "DETACHED=-d"
    call :run_stack
    timeout /t 15 >nul
)

docker exec airsim-container /home/airsim_user/Scripts/entrypoint.sh test
goto :eof

:open_shell
echo === Opening Shell in AirSim Container ===
docker ps | findstr airsim-container >nul
if errorlevel 1 (
    echo ERROR: AirSim container is not running
    echo INFO: Start the stack first: %~nx0 run
    exit /b 1
)
docker exec -it airsim-container /bin/bash
goto :eof

:show_logs
cd /d "%DOCKER_DIR%"
if "%~1"=="" (
    docker compose logs -f
) else (
    docker compose logs -f %~1
)
goto :eof

:main_execution
if "%COMMAND%"=="" set "COMMAND=run"

call :check_prerequisites
if errorlevel 1 exit /b 1

if "%COMMAND%"=="build" (
    call :build_images
) else if "%COMMAND%"=="run" (
    call :build_images
    call :run_stack
) else if "%COMMAND%"=="stop" (
    call :stop_stack
) else if "%COMMAND%"=="clean" (
    call :clean_all
) else if "%COMMAND%"=="status" (
    call :show_status
) else if "%COMMAND%"=="test" (
    call :run_tests
) else if "%COMMAND%"=="shell" (
    call :open_shell
) else if "%COMMAND%"=="logs" (
    call :show_logs
) else (
    echo ERROR: Unknown command: %COMMAND%
    goto show_help
)

:end 