"""
Status command implementation for MNSTEVV CLI

Provides comprehensive health monitoring and status checks for all services.
"""
import click
import sys
import json
import subprocess
import socket
from pathlib import Path

# Add core modules to path
core_path = str(Path(__file__).parent.parent / 'core')
if core_path not in sys.path:
    sys.path.insert(0, core_path)

try:
    from docker_compose import DockerComposeWrapper
except ImportError as e:
    click.echo(f"ERROR: Failed to import core modules: {e}")
    click.echo("Make sure the mnstevv package is properly installed.")
    sys.exit(1)

def check_port_connectivity(host: str, port: int, timeout: int = 3) -> bool:
    """Check if a port is reachable."""
    try:
        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        sock.settimeout(timeout)
        result = sock.connect_ex((host, port))
        sock.close()
        return result == 0
    except Exception:
        return False

@click.command('status')
@click.option('--profile', '-p', default='integrated', 
              type=click.Choice(['integrated', 'px4-only', 'ros2-only', 'development']),
              help='Deployment profile to check [default: integrated]')
@click.option('--detailed', '-d', is_flag=True, default=False,
              help='Show detailed service information')
@click.option('--json', 'json_output', is_flag=True, default=False,
              help='Output status in JSON format')
@click.pass_context
def status_command(ctx, profile, detailed, json_output):
    """
    Show comprehensive service health status.
    
    The status command provides real-time information about running services,
    their health status, network connectivity, and port availability.
    
    \b
    Examples:
        mnstevv status                    # Basic status overview
        mnstevv status --detailed         # Detailed service information
        mnstevv status --json             # JSON output for automation
        mnstevv status --profile px4-only # Status for specific profile
    """
    try:
        compose_file = ctx.obj['compose_file']
        docker_compose = DockerComposeWrapper(compose_file)
        
        # Get running services
        running_services = docker_compose.get_running_services(profile=profile)
        
        if json_output:
            # JSON output for automation
            status_data = {
                'profile': profile,
                'services': running_services,
                'total_services': len(running_services),
                'healthy_services': len([s for s in running_services if s.get('State') == 'running'])
            }
            click.echo(json.dumps(status_data, indent=2))
            return
        
        # Human-readable output
        click.echo(f"MNSTEVV Service Status - Profile: {profile}")
        click.echo("=" * 50)
        
        if not running_services:
            click.echo("No services are currently running")
            click.echo(f"\nStart services with: mnstevv up --num_drones 3")
            return
        
        # Categorize services by status
        running = [s for s in running_services if s.get('State') == 'running']
        stopped = [s for s in running_services if s.get('State') != 'running']
        
        click.echo(f"Running: {len(running)}")
        click.echo(f"Stopped: {len(stopped)}")
        click.echo(f"Total:   {len(running_services)}")
        click.echo()
        
        # Show running services
        if running:
            click.echo("Running Services:")
            for service in running:
                name = service.get('Name', 'unknown')
                status = service.get('State', 'unknown')
                ports = service.get('Ports', '')
                
                status_icon = "[RUNNING]" if status == "running" else "[WARNING]"
                click.echo(f"   {status_icon} {name}: {status}")
                
                if ports and detailed:
                    click.echo(f"      Ports: {ports}")
                
                # Additional health checks for specific services
                if detailed:
                    if 'ros2-multi-node' in name:
                        click.echo("      ROS2 multi-vehicle coordination")
                        # Could add ROS2 topic checks here
                    elif 'px4-bridge-drone' in name:
                        click.echo("      PX4 SITL drone instance")
                        # Could add MAVLink connectivity checks here
                    elif 'ecosystem-monitor' in name:
                        click.echo("      Cross-ecosystem monitoring")
        
        # Show stopped services
        if stopped:
            click.echo(f"\nStopped Services:")
            for service in stopped:
                name = service.get('Name', 'unknown')
                status = service.get('State', 'unknown')
                click.echo(f"   [STOPPED] {name}: {status}")
        
        # Network connectivity checks
        if detailed and running:
            click.echo(f"\nNetwork Connectivity:")
            
            # Check common AirSim ports
            airsim_hosts = ['host.docker.internal', 'localhost', '172.18.144.1']
            airsim_port = 41451
            
            airsim_reachable = False
            for host in airsim_hosts:
                if check_port_connectivity(host, airsim_port):
                    click.echo(f"   [OK] AirSim API: {host}:{airsim_port}")
                    airsim_reachable = True
                    break
            
            if not airsim_reachable:
                click.echo(f"   [FAIL] AirSim API: Not reachable on port {airsim_port}")
                click.echo(f"      Make sure AirSim is running on Windows")
            
            # Check for PX4 MAVLink ports (if PX4 services are running)
            px4_services = [s for s in running if 'px4-bridge-drone' in s.get('Name', '')]
            if px4_services:
                click.echo(f"   MAVLink Connectivity:")
                for i, service in enumerate(px4_services, 1):
                    mavlink_port = 4559 + i  # PX4 MAVLink ports start at 4560
                    if check_port_connectivity('localhost', mavlink_port):
                        click.echo(f"      [OK] Drone {i}: localhost:{mavlink_port}")
                    else:
                        click.echo(f"      [FAIL] Drone {i}: localhost:{mavlink_port} not reachable")
        
        # Show quick action suggestions
        if not detailed:
            click.echo(f"\nQuick Actions:")
            click.echo(f"   mnstevv status --detailed    # More detailed status")
            click.echo(f"   mnstevv logs --follow        # Follow service logs")
            if not running:
                click.echo(f"   mnstevv up --num_drones 3    # Start 3 drone services")
            else:
                click.echo(f"   mnstevv down                 # Stop all services")
    
    except Exception as e:
        if json_output:
            error_data = {'error': str(e), 'profile': profile}
            click.echo(json.dumps(error_data, indent=2))
        else:
            click.echo(f"ERROR: Error getting status: {e}", err=True)
        sys.exit(1)