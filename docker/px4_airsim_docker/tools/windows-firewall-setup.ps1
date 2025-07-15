# Windows Firewall Setup for AirSim Ultra-Swarm
# Run this script in PowerShell as Administrator

Write-Host "Setting up Windows Firewall for AirSim Ultra-Swarm..." -ForegroundColor Green

# AirSim API port
New-NetFirewallRule -DisplayName "AirSim API" -Direction Inbound -Protocol TCP -LocalPort 41451 -Action Allow -ErrorAction SilentlyContinue

# PX4 TCP ports for 9 drones
$startPort = 4561
$endPort = 4569
New-NetFirewallRule -DisplayName "AirSim PX4 TCP Ports" -Direction Inbound -Protocol TCP -LocalPort $startPort-$endPort -Action Allow -ErrorAction SilentlyContinue

# MAVLink UDP ports
New-NetFirewallRule -DisplayName "AirSim MAVLink UDP 14550-14558" -Direction Inbound -Protocol UDP -LocalPort 14550-14558 -Action Allow -ErrorAction SilentlyContinue
New-NetFirewallRule -DisplayName "AirSim MAVLink UDP 18570-18578" -Direction Inbound -Protocol UDP -LocalPort 18570-18578 -Action Allow -ErrorAction SilentlyContinue

Write-Host "Firewall rules created successfully!" -ForegroundColor Green
Write-Host ""
Write-Host "Current AirSim-related firewall rules:" -ForegroundColor Yellow
Get-NetFirewallRule -DisplayName "AirSim*" | Format-Table DisplayName, Enabled, Direction, Protocol

Write-Host ""
Write-Host "To remove these rules later, run:" -ForegroundColor Cyan
Write-Host 'Get-NetFirewallRule -DisplayName "AirSim*" | Remove-NetFirewallRule'