"""
Logs command implementation for MNSTEVV CLI

Provides centralized log viewing with filtering and aggregation capabilities.
"""
import click
import sys
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

@click.command('logs')
@click.argument('services', nargs=-1)
@click.option('--profile', '-p', default='integrated', 
              type=click.Choice(['integrated', 'px4-only', 'ros2-only', 'development']),
              help='Deployment profile [default: integrated]')
@click.option('--follow', '-f', is_flag=True, default=False,
              help='Follow log output (like tail -f)')
@click.option('--tail', type=int, default=50,
              help='Number of lines from end of logs [default: 50]')
@click.option('--timestamps', '-t', is_flag=True, default=False,
              help='Show timestamps in log output')
@click.option('--filter', 'log_filter', default=None,
              help='Filter logs by pattern (grep-style)')
@click.pass_context
def logs_command(ctx, services, profile, follow, tail, timestamps, log_filter):
    """
    View service logs with filtering and aggregation.
    
    The logs command provides centralized access to all service logs with
    powerful filtering and follow capabilities. If no services are specified,
    logs from all running services are shown.
    
    \b
    Arguments:
        SERVICES    Specific service names to view logs from (optional)
    
    \b
    Examples:
        mnstevv logs                              # All service logs
        mnstevv logs ros2-multi-node              # Specific service logs
        mnstevv logs px4-bridge-drone-1 px4-bridge-drone-2  # Multiple services
        mnstevv logs --follow                     # Follow all logs live
        mnstevv logs --filter "ERROR"             # Filter by pattern
        mnstevv logs --tail 100 --timestamps      # Last 100 lines with timestamps
    """
    try:
        compose_file = ctx.obj['compose_file']
        docker_compose = DockerComposeWrapper(compose_file)
        
        # Convert services tuple to list
        service_list = list(services) if services else None
        
        # Show configuration
        if service_list:
            click.echo(f"Viewing logs for: {', '.join(service_list)}")
        else:
            click.echo(f"Viewing logs for all services (profile: {profile})")
        
        if log_filter:
            click.echo(f"Filtering by: {log_filter}")
        
        if follow:
            click.echo(f"Following logs (Ctrl+C to stop)...")
        else:
            click.echo(f"Showing last {tail} lines...")
        
        click.echo("=" * 50)
        
        try:
            # Get logs using docker-compose
            if log_filter:
                # For filtering, we need to capture output and pipe through grep
                import subprocess
                
                # Build docker-compose logs command
                cmd = ['docker-compose', '-f', str(compose_file), '--profile', profile, 'logs']
                
                if follow:
                    cmd.append('-f')
                if timestamps:
                    cmd.append('-t')
                if tail and not follow:  # tail doesn't work well with follow
                    cmd.extend(['--tail', str(tail)])
                
                if service_list:
                    cmd.extend(service_list)
                
                # Use subprocess to pipe through grep for filtering
                compose_process = subprocess.Popen(
                    cmd,
                    stdout=subprocess.PIPE,
                    stderr=subprocess.STDOUT,
                    text=True,
                    cwd=ctx.obj['docker_dir']
                )
                
                grep_process = subprocess.Popen(
                    ['grep', '--line-buffered', log_filter],
                    stdin=compose_process.stdout,
                    stdout=subprocess.PIPE,
                    stderr=subprocess.STDOUT,
                    text=True
                )
                
                compose_process.stdout.close()
                
                try:
                    for line in iter(grep_process.stdout.readline, ''):
                        if line:
                            click.echo(line.rstrip())
                    grep_process.wait()
                except KeyboardInterrupt:
                    if follow:
                        click.echo(f"\nStopped following logs")
                    grep_process.terminate()
                    compose_process.terminate()
            else:
                # No filtering - use docker-compose wrapper directly
                result = docker_compose.logs(
                    services=service_list,
                    profile=profile,
                    follow=follow,
                    tail=tail if not follow else None,
                    timestamps=timestamps
                )
                
                if not follow and result.stdout:
                    click.echo(result.stdout)
        
        except KeyboardInterrupt:
            if follow:
                click.echo(f"\n✋ Stopped following logs")
            else:
                click.echo(f"\nInterrupted")
        except Exception as e:
            click.echo(f"ERROR: Error retrieving logs: {e}", err=True)
            sys.exit(1)
    
    except Exception as e:
        click.echo(f"ERROR: Error with logs command: {e}", err=True)
        sys.exit(1)