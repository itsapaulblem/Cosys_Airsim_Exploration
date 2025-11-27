#!/usr/bin/env python3
"""
MNSTEVV - Multi-Node System Test Environment Vehicle Validator
Main CLI entry point using Click framework.
"""
import click
import os
import sys
from pathlib import Path

# Add current directory to Python path for imports
current_dir = os.path.dirname(os.path.abspath(__file__))
if current_dir not in sys.path:
    sys.path.insert(0, current_dir)

try:
    from commands.up import up_command
    from commands.down import down_command
    from commands.stop import stop_command
    from commands.restart import restart_command
    from commands.status import status_command
    from commands.ps import ps_command
    from commands.logs import logs_command
    from commands.config import config_command
    from commands.clean import clean_command
    from commands.postgres import postgres_command
except ImportError as e:
    click.echo(f"ERROR: Failed to import commands: {e}")
    click.echo("Make sure you're in the correct directory and all dependencies are installed.")
    sys.exit(1)

@click.group()
@click.version_option(version='1.0.0', prog_name='mnstevv')
@click.option('--monitoring', default=False, is_flag=False, flag_value=True, type=click.BOOL,
              help='Enable monitoring profile: use --monitoring or --monitoring=true [default: false]')
@click.option('--platform', type=click.Choice(['linux', 'windows']),
              help='Platform-specific configuration (adds hybrid-override and platform profile)')
@click.option('--yolo', default=None, is_flag=False, flag_value=True, type=click.BOOL,
              help='Enable YOLOv10 detection service: auto-enabled with --platform linux [default: auto]')
@click.option('--qgc', default=None, is_flag=False, flag_value=True, type=click.BOOL,
              help='Enable QGroundControl X11: auto-enabled with --platform linux [default: auto]')
@click.pass_context
def cli(ctx, monitoring, platform, yolo, qgc):
    """
    MNSTEVV - Multi-Node System Test Environment Vehicle Validator

    A CLI tool for managing AirSim Docker Compose ecosystems with intelligent
    service selection based on drone count and deployment profiles.

    Examples:
    \b
        mnstevv up --num_drones 3                    # Start 3 drones (integrated profile)
        mnstevv --monitoring --platform linux up     # Enable monitoring + Linux platform
        mnstevv up --num_drones 2 --profile px4-only # Start 2 PX4 drones only
        mnstevv up --include-postgres                # Include PostgreSQL services
        mnstevv postgres up                          # Start only PostgreSQL services
        mnstevv status                               # Show service health
        mnstevv logs --follow                        # Follow aggregated logs
        mnstevv clean                                # Clean up all resources
    """
    # Ensure the context object exists
    ctx.ensure_object(dict)

    # Store the docker directory path for commands to use
    script_dir = Path(__file__).parent.parent.parent
    ctx.obj['docker_dir'] = script_dir
    ctx.obj['compose_file'] = script_dir / 'docker-compose-master.yml'

    # Store global options in context for subcommands to use
    ctx.obj['monitoring'] = monitoring
    ctx.obj['platform'] = platform
    ctx.obj['yolo'] = yolo
    ctx.obj['qgc'] = qgc

# Register commands
cli.add_command(up_command, name='up')
cli.add_command(down_command, name='down')
cli.add_command(stop_command, name='stop')
cli.add_command(restart_command, name='restart')
cli.add_command(status_command, name='status')
cli.add_command(ps_command, name='ps')
cli.add_command(logs_command, name='logs')
cli.add_command(config_command, name='config')
cli.add_command(clean_command, name='clean')
cli.add_command(postgres_command, name='postgres')

if __name__ == '__main__':
    cli()