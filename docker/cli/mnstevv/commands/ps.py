"""
PS command implementation for MNSTEVV CLI

Provides quick container listing with status information.
"""
import click
import sys
import json
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

@click.command('ps')
@click.option('--profile', '-p', default='integrated', 
              type=click.Choice(['integrated', 'px4-only', 'ros2-only', 'development']),
              help='Deployment profile [default: integrated]')
@click.option('--all', '-a', is_flag=True, default=False,
              help='Show all containers (including stopped)')
@click.option('--json', 'json_output', is_flag=True, default=False,
              help='Output in JSON format')
@click.option('--filter', 'service_filter', default=None,
              help='Filter services by name pattern')
@click.pass_context
def ps_command(ctx, profile, all, json_output, service_filter):
    """
    List containers with their current status.
    
    This provides a quick overview of running containers, similar to 'docker ps'
    but focused on the services managed by the compose file.
    
    Examples:
        mnstevv ps                              # List running containers
        mnstevv ps --all                        # List all containers (including stopped)
        mnstevv ps --json                       # JSON output for scripting
        mnstevv ps --filter px4                 # Filter containers by name pattern
        mnstevv ps --profile px4-only           # List PX4-only containers
    """
    try:
        compose_file = ctx.obj['compose_file']
        docker_compose = DockerComposeWrapper(compose_file)
        
        # Get container information
        try:
            cmd = ['ps']
            if all:
                cmd.append('-a')
            if json_output:
                cmd.extend(['--format', 'json'])
            
            result = docker_compose.execute(cmd, profile=profile, capture_output=True)
            
            if json_output:
                # Parse and filter JSON output
                if result.stdout.strip():
                    containers = []
                    for line in result.stdout.strip().split('\n'):
                        if line.strip():
                            container = json.loads(line.strip())
                            if not service_filter or service_filter.lower() in container.get('Name', '').lower():
                                containers.append(container)
                    
                    if service_filter and not containers:
                        click.echo(json.dumps({"error": f"No containers found matching filter: {service_filter}"}))
                    else:
                        click.echo(json.dumps(containers, indent=2))
                else:
                    click.echo(json.dumps([]))
            else:
                # Human-readable output
                click.echo(f"MNSTEVV Container Status - Profile: {profile}")
                click.echo("=" * 60)
                
                if result.stdout.strip():
                    # Parse JSON output for human display
                    containers = []
                    for line in result.stdout.strip().split('\n'):
                        if line.strip():
                            try:
                                container = json.loads(line.strip())
                                if not service_filter or service_filter.lower() in container.get('Name', '').lower():
                                    containers.append(container)
                            except json.JSONDecodeError:
                                # Fallback: treat as regular table output
                                if not service_filter or service_filter.lower() in line.lower():
                                    click.echo(line)
                                continue
                    
                    if not containers and service_filter:
                        click.echo(f"No containers found matching filter: {service_filter}")
                        return
                    
                    if containers:
                        # Display formatted table
                        click.echo(f"{'NAME':<20} {'SERVICE':<20} {'STATUS':<15} {'HEALTH':<10} {'PORTS':<30}")
                        click.echo("-" * 95)
                        
                        for container in containers:
                            name = container.get('Name', 'unknown')[:19]
                            service = container.get('Service', 'unknown')[:19] 
                            status = container.get('Status', 'unknown')[:14]
                            health = container.get('Health', '')[:9]
                            
                            # Simplify port display
                            ports = container.get('Ports', '')
                            if ports and len(ports) > 29:
                                # Extract key ports (like 4560, 5901, etc.)
                                import re
                                key_ports = re.findall(r'(\d{4,5})->', ports)
                                if key_ports:
                                    ports = ','.join(key_ports[:3])
                                    if len(key_ports) > 3:
                                        ports += f',+{len(key_ports)-3}'
                                else:
                                    ports = ports[:29]
                            
                            click.echo(f"{name:<20} {service:<20} {status:<15} {health:<10} {ports:<30}")
                        
                        click.echo(f"\nTotal: {len(containers)} container{'s' if len(containers) != 1 else ''}")
                    else:
                        # Try fallback to table format
                        click.echo(result.stdout if result.stdout.strip() else "No containers found")
                else:
                    click.echo("No containers found")
                    
                # Show quick actions
                if not json_output:
                    click.echo(f"\nQuick Actions:")
                    click.echo(f"   mnstevv status --detailed    # Detailed health information")
                    click.echo(f"   mnstevv logs [service]       # View service logs")
                    if not result.stdout.strip():
                        click.echo(f"   mnstevv up --num_drones 3    # Start services")
                        
        except json.JSONDecodeError as e:
            if json_output:
                click.echo(json.dumps({"error": f"Failed to parse container information: {e}"}))
            else:
                click.echo(f"ERROR: Failed to parse container information: {e}", err=True)
            sys.exit(1)
            
    except Exception as e:
        if json_output:
            click.echo(json.dumps({"error": str(e)}))
        else:
            click.echo(f"ERROR: Error listing containers: {e}", err=True)
        sys.exit(1)