@echo off
echo 🚁 PX4 Console Access via Docker
echo.

if "%1"=="" (
    echo 📡 Starting interactive PX4 shell...
    docker exec -it px4-single /Scripts/px4_shell.sh
) else (
    echo 🚀 Executing PX4 command: %*
    docker exec px4-single /Scripts/px4_shell.sh %*
)

echo.
echo 💡 Tip: Use 'docker logs px4-single --tail 20' to see detailed output 