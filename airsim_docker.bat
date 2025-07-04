@echo off
setlocal enabledelayedexpansion

REM AirSim ROS2 Docker Management Script - Windows Version
REM This script helps build and run the AirSim ROS2 Docker container

REM Default configuration
set "CONTAINER_NAME=airsim-vnc-ros2"
set "IMAGE_NAME=airsim-vnc-ros2"
set "IMAGE_TAG=improved"
set "DOCKERFILE_PATH=.\docker\airsim_ros2_wrapper\VNC\Dockerfile.ros2_vnc"
set "BUILD_CONTEXT=."
set "VNC_PORT=5901"
set "ROS_DOMAIN_ID=0"
set "AIRSIM_HOST_IP=host.docker.internal"
set "AIRSIM_HOST_PORT=41451"
set "LAUNCH_RVIZ=false"
set "ENABLE_VOLUME_MOUNT=true"

REM Command line options
set "COMMAND="
set "NO_CACHE="
set "DETACHED="
set "PRIVILEGED="
set "FOLLOW_LOGS="

REM ANSI color codes (work in Windows 10+ and Windows Terminal)
set "RED=[31m"
set "GREEN=[32m"
set "YELLOW=[33m"
set "BLUE=[34m"
set "NC=[0m"

REM Parse command line arguments
:parse_args
if "%~1"=="" goto :main

if /i "%~1"=="build" (
    set "COMMAND=build"
    shift & goto :parse_args
)
if /i "%~1"=="run" (
    set "COMMAND=run"
    shift & goto :parse_args
)
if /i "%~1"=="stop" (
    set "COMMAND=stop"
    shift & goto :parse_args
)
if /i "%~1"=="restart" (
    set "COMMAND=restart"
    shift & goto :parse_args
)
if /i "%~1"=="logs" (
    set "COMMAND=logs"
    shift & goto :parse_args
)
if /i "%~1"=="shell" (
    set "COMMAND=shell"
    shift & goto :parse_args
)
if /i "%~1"=="clean" (
    set "COMMAND=clean"
    shift & goto :parse_args
)
if /i "%~1"=="status" (
    set "COMMAND=status"
    shift & goto :parse_args
)
if /i "%~1"=="help" (
    set "COMMAND=help"
    shift & goto :parse_args
)
if /i "%~1"=="--name" (
    set "CONTAINER_NAME=%~2"
    shift & shift & goto :parse_args
)
if /i "%~1"=="--image" (
    set "IMAGE_NAME=%~2"
    shift & shift & goto :parse_args
)
if /i "%~1"=="--tag" (
    set "IMAGE_TAG=%~2"
    shift & shift & goto :parse_args
)
if /i "%~1"=="--dockerfile" (
    set "DOCKERFILE_PATH=%~2"
    shift & shift & goto :parse_args
)
if /i "%~1"=="--context" (
    set "BUILD_CONTEXT=%~2"
    shift & shift & goto :parse_args
)
if /i "%~1"=="--vnc-port" (
    set "VNC_PORT=%~2"
    shift & shift & goto :parse_args
)
if /i "%~1"=="--ros-domain-id" (
    set "ROS_DOMAIN_ID=%~2"
    shift & shift & goto :parse_args
)
if /i "%~1"=="--airsim-host" (
    set "AIRSIM_HOST_IP=%~2"
    shift & shift & goto :parse_args
)
if /i "%~1"=="--airsim-port" (
    set "AIRSIM_HOST_PORT=%~2"
    shift & shift & goto :parse_args
)
if /i "%~1"=="--launch-rviz" (
    set "LAUNCH_RVIZ=true"
    shift & goto :parse_args
)
if /i "%~1"=="--no-cache" (
    set "NO_CACHE=--no-cache"
    shift & goto :parse_args
)
if /i "%~1"=="--detach" (
    set "DETACHED=-d"
    shift & goto :parse_args
)
if /i "%~1"=="--privileged" (
    set "PRIVILEGED=--privileged"
    shift & goto :parse_args
)
if /i "%~1"=="--follow" (
    set "FOLLOW_LOGS=-f"
    shift & goto :parse_args
)
if /i "%~1"=="--no-volume-mount" (
    set "ENABLE_VOLUME_MOUNT=false"
    shift & goto :parse_args
)

call :print_error "Unknown option: %~1"
call :show_usage
exit /b 1

:main
call :check_docker
if errorlevel 1 exit /b 1

if "%COMMAND%"=="" call :show_usage & exit /b 0
if /i "%COMMAND%"=="help" call :show_usage & exit /b 0

if /i "%COMMAND%"=="build" call :build_image
if /i "%COMMAND%"=="run" call :run_container
if /i "%COMMAND%"=="stop" call :stop_container
if /i "%COMMAND%"=="restart" call :restart_container
if /i "%COMMAND%"=="logs" call :show_logs
if /i "%COMMAND%"=="shell" call :open_shell
if /i "%COMMAND%"=="clean" call :clean_up
if /i "%COMMAND%"=="status" call :show_status

exit /b 0

:show_usage
echo Usage: %~nx0 [COMMAND] [OPTIONS]
echo.
echo Commands:
echo     build       Build the Docker image
echo     run         Run the Docker container
echo     stop        Stop the running container
echo     restart     Restart the container
echo     logs        Show container logs
echo     shell       Open a shell in the running container
echo     clean       Remove container and image
echo     status      Show container status
echo     help        Show this help message
echo.
echo Options:
echo     --name NAME             Container name (default: %CONTAINER_NAME%)
echo     --image IMAGE           Image name (default: %IMAGE_NAME%)
echo     --tag TAG               Image tag (default: %IMAGE_TAG%)
echo     --dockerfile PATH       Dockerfile path (default: %DOCKERFILE_PATH%)
echo     --context PATH          Build context path (default: %BUILD_CONTEXT%)
echo     --vnc-port PORT         VNC port (default: %VNC_PORT%)
echo     --ros-domain-id ID      ROS Domain ID (default: %ROS_DOMAIN_ID%)
echo     --airsim-host HOST      AirSim host IP (default: %AIRSIM_HOST_IP%)
echo     --airsim-port PORT      AirSim host port (default: %AIRSIM_HOST_PORT%)
echo     --launch-rviz           Launch RViz2 with AirSim node
echo     --no-cache              Build without cache
echo     --detach                Run container in detached mode
echo     --privileged            Run container in privileged mode
echo     --no-volume-mount       Disable volume mounting of ros2 folder
echo.
echo Examples:
echo     %~nx0 build --no-cache
echo     %~nx0 run --launch-rviz --vnc-port 8080
echo     %~nx0 run --airsim-host 192.168.1.100 --airsim-port 41451
echo     %~nx0 shell
echo     %~nx0 logs --follow
echo.
echo Development Workflow:
echo     1. %~nx0 build           # Build the Docker image with development tools
echo     2. %~nx0 run --detach    # Start container with volume mounting enabled
echo     3. Connect to VNC at http://localhost:%VNC_PORT% (password: ubuntu)
echo     4. Use aliases in container: build, build_interfaces, source_ws
echo     5. Edit files in local IDE - changes reflect immediately in container
echo.
exit /b 0

:print_status
echo %GREEN%[INFO]%NC% %~1
exit /b 0

:print_warning
echo %YELLOW%[WARNING]%NC% %~1
exit /b 0

:print_error
echo %RED%[ERROR]%NC% %~1
exit /b 0

:print_header
echo %BLUE%================================%NC%
echo %BLUE%%~1%NC%
echo %BLUE%================================%NC%
exit /b 0

:check_docker
docker info >nul 2>&1
if errorlevel 1 (
    call :print_error "Docker is not running. Please start Docker and try again."
    exit /b 1
)
exit /b 0

:build_image
call :print_header "Building AirSim ROS2 Docker Image"

if not exist "%DOCKERFILE_PATH%" (
    call :print_error "Dockerfile not found at: %DOCKERFILE_PATH%"
    exit /b 1
)

call :print_status "Building image: %IMAGE_NAME%:%IMAGE_TAG%"
call :print_status "Using Dockerfile: %DOCKERFILE_PATH%"
call :print_status "Build context: %BUILD_CONTEXT%"

docker build %NO_CACHE% -f "%DOCKERFILE_PATH%" -t "%IMAGE_NAME%:%IMAGE_TAG%" "%BUILD_CONTEXT%"
if errorlevel 1 (
    call :print_error "Build failed!"
    exit /b 1
)

call :print_status "Build completed successfully!"
exit /b 0

:run_container
call :print_header "Running AirSim ROS2 Container"

REM Setup volume mounting
if "%ENABLE_VOLUME_MOUNT%"=="true" (
    if exist "ros2\src" (
        set "VOLUME_ARGS=-v "%CD%\ros2\src":/airsim_ros2_ws/src -v "%CD%\AirLib":/airsim_ros2_ws/AirLib -v "%CD%\MavLinkCom":/airsim_ros2_ws/MavLinkCom"
        call :print_status "Volume mounting enabled - local changes will reflect immediately"
    ) else (
        call :print_warning "ros2\src directory not found - volume mounting disabled"
        set "VOLUME_ARGS="
    )
) else (
    set "VOLUME_ARGS="
    call :print_status "Volume mounting disabled"
)

REM Check if container already exists
docker ps -a --format "table {{.Names}}" | findstr /r /c:"^%CONTAINER_NAME%$" >nul 2>&1
if not errorlevel 1 (
    call :print_warning "Container '%CONTAINER_NAME%' already exists."
    set /p "REPLY=Do you want to remove it and create a new one? (y/N): "
    if /i "!REPLY!"=="y" (
        docker rm -f "%CONTAINER_NAME%" >nul 2>&1
    ) else (
        call :print_status "Starting existing container..."
        docker start "%CONTAINER_NAME%"
        exit /b 0
    )
)

call :print_status "Starting new container: %CONTAINER_NAME%"
call :print_status "VNC will be available at: localhost:%VNC_PORT%"
call :print_status "ROS Domain ID: %ROS_DOMAIN_ID%"
call :print_status "AirSim Host: %AIRSIM_HOST_IP%:%AIRSIM_HOST_PORT%"
call :print_status "Launch RViz2: %LAUNCH_RVIZ%"

REM Create and run the container
docker run %DETACHED% %PRIVILEGED% --name "%CONTAINER_NAME%" -p "%VNC_PORT%:5901" -p 7400:7400/udp -p 7401:7401/udp -p 7402:7402/udp %VOLUME_ARGS% -e ROS_DOMAIN_ID="%ROS_DOMAIN_ID%" -e AIRSIM_HOST_IP="%AIRSIM_HOST_IP%" -e AIRSIM_HOST_PORT="%AIRSIM_HOST_PORT%" -e LAUNCH_RVIZ="%LAUNCH_RVIZ%" -e DISPLAY=:1 --shm-size=1g "%IMAGE_NAME%:%IMAGE_TAG%"

if "%DETACHED%"=="" (
    call :print_status "Container is running in foreground mode."
) else (
    call :print_status "Container started in detached mode."
    call :print_status "Access VNC at: http://localhost:%VNC_PORT%"
    call :print_status "Default VNC password: ubuntu"
    
    if "%ENABLE_VOLUME_MOUNT%"=="true" (
        echo.
        call :print_status "=== DEVELOPMENT WORKFLOW ==="
        call :print_status "Files in ros2/src are mounted - edit locally, build in container"
        call :print_status "In container terminal, use these aliases:"
        call :print_status "  build           - Build entire workspace"
        call :print_status "  build_interfaces - Build only interfaces package"
        call :print_status "  build_pkgs      - Build only ROS packages"
        call :print_status "  source_ws       - Source the workspace"
        call :print_status "  clean_build     - Clean and rebuild everything"
    )
)
exit /b 0

:stop_container
call :print_header "Stopping Container"

docker ps --format "table {{.Names}}" | findstr /r /c:"^%CONTAINER_NAME%$" >nul 2>&1
if not errorlevel 1 (
    call :print_status "Stopping container: %CONTAINER_NAME%"
    docker stop "%CONTAINER_NAME%"
    call :print_status "Container stopped successfully!"
) else (
    call :print_warning "Container '%CONTAINER_NAME%' is not running."
)
exit /b 0

:restart_container
call :print_header "Restarting Container"

docker ps -a --format "table {{.Names}}" | findstr /r /c:"^%CONTAINER_NAME%$" >nul 2>&1
if not errorlevel 1 (
    call :print_status "Restarting container: %CONTAINER_NAME%"
    docker restart "%CONTAINER_NAME%"
    call :print_status "Container restarted successfully!"
    call :print_status "Access VNC at: http://localhost:%VNC_PORT%"
) else (
    call :print_warning "Container '%CONTAINER_NAME%' does not exist."
    call :print_status "Use '%~nx0 run' to create and start a new container."
)
exit /b 0

:show_logs
call :print_header "Container Logs"

docker ps -a --format "table {{.Names}}" | findstr /r /c:"^%CONTAINER_NAME%$" >nul 2>&1
if not errorlevel 1 (
    docker logs %FOLLOW_LOGS% "%CONTAINER_NAME%"
) else (
    call :print_warning "Container '%CONTAINER_NAME%' does not exist."
)
exit /b 0

:open_shell
call :print_header "Opening Shell in Container"

docker ps --format "table {{.Names}}" | findstr /r /c:"^%CONTAINER_NAME%$" >nul 2>&1
if not errorlevel 1 (
    call :print_status "Opening cmd shell in container: %CONTAINER_NAME%"
    docker exec -it "%CONTAINER_NAME%" /bin/bash
) else (
    call :print_warning "Container '%CONTAINER_NAME%' is not running."
    call :print_status "Use '%~nx0 run' to start the container first."
)
exit /b 0

:clean_up
call :print_header "Cleaning Up"

REM Stop and remove container
docker ps -a --format "table {{.Names}}" | findstr /r /c:"^%CONTAINER_NAME%$" >nul 2>&1
if not errorlevel 1 (
    call :print_status "Removing container: %CONTAINER_NAME%"
    docker rm -f "%CONTAINER_NAME%"
)

REM Remove image
docker images --format "table {{.Repository}}:{{.Tag}}" | findstr /r /c:"^%CONTAINER_NAME%:%IMAGE_TAG%$" >nul 2>&1
if not errorlevel 1 (
    set /p "REPLY=Do you want to remove the image '%IMAGE_NAME%:%IMAGE_TAG%'? (y/N): "
    if /i "!REPLY!"=="y" (
        call :print_status "Removing image: %IMAGE_NAME%:%IMAGE_TAG%"
        docker rmi "%IMAGE_NAME%:%IMAGE_TAG%"
    )
)

call :print_status "Cleanup completed!"
exit /b 0

:show_status
call :print_header "Container Status"

REM Check if container exists
docker ps -a --format "table {{.Names}}" | findstr /r /c:"^%CONTAINER_NAME%$" >nul 2>&1
if not errorlevel 1 (
    call :print_status "Container '%CONTAINER_NAME%' exists"
    
    REM Check if running
    docker ps --format "table {{.Names}}" | findstr /r /c:"^%CONTAINER_NAME%$" >nul 2>&1
    if not errorlevel 1 (
        call :print_status "Status: RUNNING"
        call :print_status "VNC Access: http://localhost:%VNC_PORT%"
        
        REM Show container details
        echo.
        docker ps --filter "name=%CONTAINER_NAME%" --format "table {{.Names}}\t{{.Status}}\t{{.Ports}}"
    ) else (
        call :print_warning "Status: STOPPED"
    )
) else (
    call :print_warning "Container '%CONTAINER_NAME%' does not exist"
)

REM Check if image exists
docker images --format "table {{.Repository}}:{{.Tag}}" | findstr /r /c:"^%IMAGE_NAME%:%IMAGE_TAG%$" >nul 2>&1
if not errorlevel 1 (
    call :print_status "Image '%IMAGE_NAME%:%IMAGE_TAG%' exists"
) else (
    call :print_warning "Image '%IMAGE_NAME%:%IMAGE_TAG%' does not exist"
)
exit /b 0