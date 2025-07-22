@echo off
setlocal enabledelayedexpansion

REM Optimized Bridge Network PX4 Swarm Management Script (Windows)
REM Reduced redundancy and improved efficiency
REM Compatible with Docker Desktop limitations on Windows/WSL2

REM Configuration
set "SCRIPT_DIR=%~dp0"
set "PROJECT_DIR=%SCRIPT_DIR%.."
set "COMPOSE_FILE=%PROJECT_DIR%\docker-compose-slim.yml"

REM Default PX4 simulation hostname (can be overridden)
set "DEFAULT_PX4_SIM_HOSTNAME=172.28.240.1"
set "CUSTOM_PX4_SIM_HOSTNAME="

REM Container configuration
set "DEFAULT_CONTAINERS=px4-bridge-drone-1 px4-bridge-drone-2 px4-bridge-drone-3 px4-bridge-drone-4 px4-bridge-drone-5 px4-bridge-drone-6 px4-bridge-drone-7 px4-bridge-drone-8 px4-bridge-drone-9"

REM Colors (Windows doesn't support colors the same way, using echo messages)
REM Environment cache
set "ENVIRONMENT_CACHE="
set "WSL2_IP_CACHE="

REM Function to display colored output
:log
echo [INFO] %~1
goto :eof

:warn
echo [WARN] %~1
goto :eof

:error
echo [ERROR] %~1
goto :eof

:header
echo [BRIDGE SWARM] %~1
goto :eof

REM Function to get the effective PX4_SIM_HOSTNAME
:get_px4_sim_hostname
if not "%CUSTOM_PX4_SIM_HOSTNAME%"=="" (
    set "PX4_SIM_HOST=%CUSTOM_PX4_SIM_HOSTNAME%"
) else (
    set "PX4_SIM_HOST=%DEFAULT_PX4_SIM_HOSTNAME%"
)
goto :eof

REM Connectivity test function
:test_connectivity
set "host=%~1"
set "port=%~2"
if "%port%"=="" set "port=41451"
set "timeout=%~3"
if "%timeout%"=="" set "timeout=3"

REM Use telnet for connectivity test on Windows
echo. | telnet %host% %port% 2>nul
if %errorlevel%==0 (
    exit /b 0
) else (
    exit /b 1
)

REM Environment detection
:detect_environment
if not "%ENVIRONMENT_CACHE%"=="" goto :eof

set "ENVIRONMENT_CACHE=bridge"
REM Check if running in WSL2
wsl --list --quiet >nul 2>&1
if %errorlevel%==0 (
    set "ENVIRONMENT_CACHE=wsl2"
    REM Get WSL2 IP (simplified for Windows)
    for /f "tokens=2 delims=:" %%a in ('ipconfig ^| findstr "IPv4"') do (
        set "WSL2_IP_CACHE=%%a"
        set "WSL2_IP_CACHE=!WSL2_IP_CACHE: =!"
        goto :wsl2_ip_found
    )
    :wsl2_ip_found
)
goto :eof

REM Network connectivity check
:check_network_connectivity
call :detect_environment
call :log "Bridge Network Mode - Testing connectivity..."
call :get_px4_sim_hostname

set "connectivity_ok=false"

REM Test configured PX4_SIM_HOSTNAME
call :test_connectivity "%PX4_SIM_HOST%" 41451
if %errorlevel%==0 (
    call :log "✅ AirSim API accessible via %PX4_SIM_HOST%:41451"
    set "connectivity_ok=true"
) else (
    call :warn "⚠️  AirSim API not accessible via %PX4_SIM_HOST%:41451"
)

REM Test WSL2 IP if available
if "%ENVIRONMENT_CACHE%"=="wsl2" if not "%WSL2_IP_CACHE%"=="" (
    call :test_connectivity "%WSL2_IP_CACHE%" 41451
    if !errorlevel!==0 (
        call :log "✅ AirSim also accessible via WSL2 IP: %WSL2_IP_CACHE%:41451"
        set "connectivity_ok=true"
    ) else (
        call :warn "⚠️  AirSim not accessible via WSL2 IP: %WSL2_IP_CACHE%:41451"
    )
)

if "%connectivity_ok%"=="false" (
    call :warn "Make sure AirSim is running and configured for bridge network access"
)
goto :eof

REM Get running containers
:get_running_containers
set "running_containers="
for %%c in (%DEFAULT_CONTAINERS%) do (
    docker ps --format "{{.Names}}" | findstr /r /c:"^%%c$" >nul 2>&1
    if !errorlevel!==0 (
        set "running_containers=!running_containers! %%c"
    )
)
goto :eof

REM Get container info
:get_container_info
set "container=%~1"
docker ps --format "{{.Names}}" | findstr /r /c:"^%container%$" >nul 2>&1
if %errorlevel%==0 (
    for /f "tokens=*" %%i in ('docker inspect -f "{{range .NetworkSettings.Networks}}{{.IPAddress}}{{end}}" "%container%" 2^>nul') do set "ip=%%i"
    if "!ip!"=="" set "ip=N/A"
    
    docker exec "%container%" pgrep -f "px4" >nul 2>&1
    if !errorlevel!==0 (
        set "px4_status=PX4_RUNNING"
    ) else (
        set "px4_status=CONTAINER_ONLY"
    )
    
    set "container_info=%container%|RUNNING|!ip!|!px4_status!"
) else (
    set "container_info=%container%|STOPPED|N/A|NOT_RUNNING"
)
goto :eof

REM Start containers
:start_containers
set "num_drones=%~1"
if "%num_drones%"=="" set "num_drones=9"
call :get_px4_sim_hostname

call :header "Starting %num_drones% containers in bridge network mode..."
call :log "Using PX4_SIM_HOSTNAME: %PX4_SIM_HOST%"
call :check_network_connectivity

REM Build containers list
set "containers_to_start="
set "count=0"
for %%c in (%DEFAULT_CONTAINERS%) do (
    set /a count+=1
    if !count! leq %num_drones% (
        set "containers_to_start=!containers_to_start! %%c"
    )
)

call :log "Starting containers: %containers_to_start%"

REM Set environment variable and start containers
set "PX4_SIM_HOSTNAME=%PX4_SIM_HOST%"
docker-compose -f "%COMPOSE_FILE%" up -d %containers_to_start%

call :log "Waiting for containers to be ready..."
timeout /t 5 /nobreak >nul

REM Show status
call :show_status

echo.
call :header "Port Mappings:"
for /l %%i in (1,1,%num_drones%) do (
    set /a port=4560+%%i
    call :log "Drone %%i: localhost:!port! -> container:!port!"
)

echo.
call :header "Next Steps:"
echo • Use '%~nx0 run-px4 ^<container_name^>' to start PX4 in specific container
echo • Use '%~nx0 run-all-px4' to start PX4 in all running containers
echo • Use '%~nx0 status' to check container status
goto :eof

REM Run PX4 in container
:run_px4_in_container
set "container_name=%~1"

if "%container_name%"=="" (
    call :error "Container name required. Usage: %~nx0 run-px4 <container_name>"
    goto :eof
)

docker ps --format "{{.Names}}" | findstr /r /c:"^%container_name%$" >nul 2>&1
if not %errorlevel%==0 (
    call :error "Container '%container_name%' is not running"
    goto :eof
)

call :header "Starting PX4 SITL in bridge container: %container_name%"

docker exec -d "%container_name%" bash -c "cd /px4_workspace/PX4-Autopilot && if [ -f \"/Scripts/run_ultra_swarm.sh\" ]; then echo \"🚀 Starting PX4 SITL with ultra-swarm configuration...\"; echo \"🔗 Instance: $PX4_INSTANCE, Swarm: $SWARM_ID\"; echo \"🌐 Network mode: Bridge (Windows Host IP: $PX4_SIM_HOSTNAME)\"; /Scripts/run_ultra_swarm.sh; else echo \"❌ run_ultra_swarm.sh not found in /Scripts/\"; echo \"Available scripts:\"; ls -la /Scripts/; exit 1; fi"

timeout /t 2 /nobreak >nul
call :log "✅ PX4 SITL started in %container_name%"
goto :eof

REM Run PX4 in all containers
:run_px4_in_all_containers
call :header "Starting PX4 SITL in all running bridge containers..."

call :get_running_containers
if "%running_containers%"=="" (
    call :error "No bridge swarm containers are currently running"
    goto :eof
)

call :log "Found running containers: %running_containers%"

for %%c in (%running_containers%) do (
    call :log "Starting PX4 in %%c..."
    call :run_px4_in_container "%%c"
    timeout /t 2 /nobreak >nul
)

call :log "✅ PX4 SITL started in all bridge containers"
goto :eof

REM Show status
:show_status
call :header "Bridge Swarm Container Status:"

set "running_count=0"
set "px4_running_count=0"

for %%c in (%DEFAULT_CONTAINERS%) do (
    call :get_container_info "%%c"
    
    for /f "tokens=1,2,3,4 delims=|" %%a in ("!container_info!") do (
        set "name=%%a"
        set "status=%%b"
        set "ip=%%c"
        set "px4_status=%%d"
        
        if "!status!"=="RUNNING" (
            set /a running_count+=1
            if "!px4_status!"=="PX4_RUNNING" (
                call :log "✅ !name! - CONTAINER + PX4 RUNNING (IP: !ip!)"
                set /a px4_running_count+=1
            ) else (
                call :warn "🟡 !name! - CONTAINER ONLY (IP: !ip!, PX4 not started)"
            )
        ) else (
            call :error "❌ !name! - NOT RUNNING"
        )
    )
)

echo.
call :header "Summary:"
set "total_containers=9"
echo • Running containers: %running_count%/%total_containers%
echo • PX4 instances: %px4_running_count%/%total_containers%
echo • Network mode: Bridge (Windows Host IP)
goto :eof

REM Show logs
:show_logs
set "container_name=%~1"
set "follow=%~2"

if "%container_name%"=="" (
    call :error "Container name required. Usage: %~nx0 logs <container_name> [follow]"
    goto :eof
)

if "%follow%"=="follow" (
    docker logs -f "%container_name%"
) else if "%follow%"=="-f" (
    docker logs -f "%container_name%"
) else (
    docker logs --tail 50 "%container_name%"
)
goto :eof

REM Stop containers
:stop_containers
call :header "Stopping all bridge swarm containers..."
docker-compose -f "%COMPOSE_FILE%" down
call :log "✅ All bridge containers stopped"
goto :eof

REM Delete containers
:delete_containers
call :header "Removing all bridge swarm containers, networks, and volumes..."

REM Stop and remove containers with volumes
docker-compose -f "%COMPOSE_FILE%" down --volumes --remove-orphans

REM Clean up any remaining px4-bridge containers
for /f "tokens=*" %%i in ('docker ps -aq --filter "name=px4-bridge-drone" 2^>nul') do (
    call :log "Removing remaining px4-bridge containers..."
    docker rm -f %%i
)

REM Remove px4_network if it exists
docker network ls | findstr "px4_network" >nul 2>&1
if %errorlevel%==0 (
    call :log "Removing px4_network..."
    docker network rm px4_network 2>nul
)

REM Clean up shared volumes
docker volume ls | findstr "px4-shared-data" >nul 2>&1
if %errorlevel%==0 (
    call :log "Removing px4-shared-data volume..."
    docker volume rm px4-shared-data 2>nul
)

call :log "✅ All bridge containers, networks, and volumes removed"
goto :eof

REM Test network connectivity
:test_network_connectivity
call :header "Bridge Network Connectivity Test"
call :check_network_connectivity

REM Test PX4 SITL ports
for %%p in (4561 4562 4563) do (
    call :test_connectivity "localhost" "%%p"
    if !errorlevel!==0 (
        call :log "✅ PX4 SITL (%%p): ACCESSIBLE via localhost"
    ) else (
        call :warn "⚠️  PX4 SITL (%%p): NOT ACCESSIBLE (may be normal if PX4 not started)"
    )
)
goto :eof

REM Show config
:show_config
call :header "Bridge Network Configuration"
call :detect_environment
call :get_px4_sim_hostname

if "%ENVIRONMENT_CACHE%"=="wsl2" (
    call :log "Environment: WSL2 + Docker Desktop (Bridge Network)"
    if not "%WSL2_IP_CACHE%"=="" (
        call :log "WSL2 IP: %WSL2_IP_CACHE%"
    ) else (
        call :log "WSL2 IP: Not detected"
    )
) else (
    call :log "Environment: Native Windows/Docker (Bridge Network)"
)

if exist "%COMPOSE_FILE%" (
    call :log "Compose file: %COMPOSE_FILE% (slim configuration)"
    call :log "Network: Custom bridge network (172.20.0.0/16)"
    call :log "Image: px4-airsim:slim (optimized multi-stage build)"
    call :log "Scripts: Ultra-swarm configuration with GPS"
    call :log "PX4_SIM_HOSTNAME: %PX4_SIM_HOST%"
) else (
    call :error "Compose file not found: %COMPOSE_FILE%"
)

echo.
call :show_status
goto :eof

REM Shell access
:shell_access
set "container_name=%~1"

if "%container_name%"=="" (
    call :error "Container name required. Usage: %~nx0 shell <container_name>"
    goto :eof
)

docker ps --format "{{.Names}}" | findstr /r /c:"^%container_name%$" >nul 2>&1
if not %errorlevel%==0 (
    call :error "Container '%container_name%' is not running"
    goto :eof
)

call :log "Opening shell in %container_name% (bridge network)..."
docker exec -it "%container_name%" /bin/bash
goto :eof

REM Show help
:show_help
echo Optimized Bridge Network PX4 Swarm Management Script (Windows)
echo Compatible with Docker Desktop limitations on Windows/WSL2
echo.
echo Usage: %~nx0 [--PX4-Sim-IP ^<ip^>] ^<command^> [options]
echo.
echo Global Options:
echo   --PX4-Sim-IP ^<ip^>    Override PX4_SIM_HOSTNAME (default: 172.28.240.1)
echo.
echo Container Management:
echo   start [num]          Start N containers (1-9, default: 9 drones)
echo   stop                 Stop all containers
echo   delete               Remove all containers, networks, and volumes
echo   restart [num]        Restart containers
echo.
echo PX4 Control:
echo   run-px4 ^<container^>  Start PX4 SITL in specific container (uses ultra-swarm script)
echo   run-all-px4          Start PX4 SITL in all running containers
echo.
echo Monitoring:
echo   status               Show container and PX4 status
echo   logs ^<container^> [follow]  Show container logs
echo   shell ^<container^>    Open shell in container
echo.
echo Network:
echo   test-connection      Test network connectivity to AirSim
echo   show-config          Show bridge network configuration
echo   help                 Show this help
echo.
echo Configuration:
echo   • Uses docker-compose-slim.yml with optimized images
echo   • Bridge network: px4_network (172.20.0.0/16)
echo   • Port mapping: localhost:4561-4569 → containers
echo   • Ultra-swarm scripts with GPS configuration
echo   • Default PX4_SIM_HOSTNAME: 172.28.240.1
echo.
echo Examples:
echo   %~nx0 start 5                                    # Start 5 containers (default IP)
echo   %~nx0 --PX4-Sim-IP 192.168.1.100 start 5        # Start 5 containers (custom IP)
echo   %~nx0 --PX4-Sim-IP 10.0.0.50 run-all-px4        # Start PX4 with custom AirSim IP
echo   %~nx0 show-config                                # Show current configuration
echo   %~nx0 delete                                     # Clean up everything
echo.
echo IP Override Examples:
echo   --PX4-Sim-IP 192.168.1.100    # LAN AirSim instance
echo   --PX4-Sim-IP 10.0.0.50         # Different subnet
echo   --PX4-Sim-IP 127.0.0.1         # Local testing
echo   --PX4-Sim-IP 172.28.240.1      # Default WSL2 gateway
goto :eof

REM Main command dispatcher
REM Process arguments for --PX4-Sim-IP
set "command="
set "arg2="
set "arg3="

:parse_args
if "%~1"=="" goto :dispatch
if "%~1"=="--PX4-Sim-IP" (
    if not "%~2"=="" (
        set "CUSTOM_PX4_SIM_HOSTNAME=%~2"
        call :log "Custom PX4_SIM_HOSTNAME set to: %CUSTOM_PX4_SIM_HOSTNAME%"
        shift
        shift
        goto :parse_args
    ) else (
        call :error "--PX4-Sim-IP requires an IP address"
        exit /b 1
    )
)
if "%command%"=="" (
    set "command=%~1"
    shift
    goto :parse_args
)
if "%arg2%"=="" (
    set "arg2=%~1"
    shift
    goto :parse_args
)
if "%arg3%"=="" (
    set "arg3=%~1"
    shift
    goto :parse_args
)
shift
goto :parse_args

:dispatch
if "%command%"=="" set "command=help"

if "%command%"=="start" (
    call :start_containers "%arg2%"
) else if "%command%"=="stop" (
    call :stop_containers
) else if "%command%"=="delete" (
    call :delete_containers
) else if "%command%"=="restart" (
    call :stop_containers
    timeout /t 2 /nobreak >nul
    call :start_containers "%arg2%"
) else if "%command%"=="status" (
    call :show_status
) else if "%command%"=="run-px4" (
    call :run_px4_in_container "%arg2%"
) else if "%command%"=="run-all-px4" (
    call :run_px4_in_all_containers
) else if "%command%"=="logs" (
    call :show_logs "%arg2%" "%arg3%"
) else if "%command%"=="shell" (
    call :shell_access "%arg2%"
) else if "%command%"=="test-connection" (
    call :test_network_connectivity
) else if "%command%"=="show-config" (
    call :show_config
) else if "%command%"=="help" (
    call :show_help
) else (
    call :show_help
)

REM Parse command line arguments
call :parse_args %*