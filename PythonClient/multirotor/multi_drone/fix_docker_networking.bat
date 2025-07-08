@echo off
echo 🚁 DOCKER TCP PORT FIX FOR AIRSIM
echo ================================

echo Stopping current containers...
cd /d L:\Cosys-AirSim\docker_clean\multi_drone
docker-compose down

echo.
echo Creating fixed docker-compose.yml...

(
echo version: '3.8'^

echo services:^

echo   px4-drone1:^

echo     build:^

echo       context: ../common^

echo       dockerfile: Dockerfile^

echo     container_name: px4-drone1^

echo     hostname: px4-drone1^

echo     environment:^

echo     - PX4_HOME_LAT=47.641468^

echo     - PX4_HOME_LON=-122.140165^

echo     - PX4_HOME_ALT=10^

echo     - PX4_SYS_AUTOSTART=10016^

echo     - PX4_SIM_HOSTNAME=host.docker.internal^

echo     - PX4_SIM_MODEL=iris^

echo     - PX4_INSTANCE=1^

echo     ports:^

echo     - "14550:14550/udp"^

echo     - "14541:14541/udp"^

echo     - "14581:14581/udp"^

echo     - "4561:4561/tcp"^

echo     networks:^

echo       airsim-network:^

echo         ipv4_address: 172.30.0.10^

echo     volumes:^

echo     - px4-shared-data:/px4_data^

echo     - ../common/scripts:/scripts^

echo     restart: unless-stopped^

echo     command:^

echo     - /Scripts/run_airsim_sitl_final.sh^

echo     - '1'^

echo.^

echo   px4-drone2:^

echo     build:^

echo       context: ../common^

echo       dockerfile: Dockerfile^

echo     container_name: px4-drone2^

echo     hostname: px4-drone2^

echo     environment:^

echo     - PX4_HOME_LAT=47.641468^

echo     - PX4_HOME_LON=-122.140165^

echo     - PX4_HOME_ALT=10^

echo     - PX4_SYS_AUTOSTART=10016^

echo     - PX4_SIM_HOSTNAME=host.docker.internal^

echo     - PX4_SIM_MODEL=iris^

echo     - PX4_INSTANCE=2^

echo     ports:^

echo     - "14551:14550/udp"^

echo     - "14542:14542/udp"^

echo     - "14582:14582/udp"^

echo     - "4562:4562/tcp"^

echo     networks:^

echo       airsim-network:^

echo         ipv4_address: 172.30.0.11^

echo     volumes:^

echo     - px4-shared-data:/px4_data^

echo     - ../common/scripts:/scripts^

echo     restart: unless-stopped^

echo     command:^

echo     - /Scripts/run_airsim_sitl_final.sh^

echo     - '2'^

echo.^

echo networks:^

echo   airsim-network:^

echo     driver: bridge^

echo     ipam:^

echo       config:^

echo       - subnet: 172.30.0.0/16^

echo.^

echo volumes:^

echo   px4-shared-data:^

echo     driver: local^

) > docker-compose-tcp-fixed.yml

echo Fixed docker-compose.yml created!
echo.
echo Starting containers with TCP ports...
docker-compose -f docker-compose-tcp-fixed.yml up -d

echo.
echo Waiting for containers to start...
timeout /t 10 /nobreak > nul

echo.
echo Testing connectivity...
docker ps --format "table {{.Names}}\t{{.Ports}}"

echo.
echo ✅ DOCKER NETWORKING FIX COMPLETE!
echo 🚁 Try your mission now: python multi_drone_orbit_mission.py
echo.
echo If GPS issues persist, restart AirSim and wait 10 seconds before testing.

pause