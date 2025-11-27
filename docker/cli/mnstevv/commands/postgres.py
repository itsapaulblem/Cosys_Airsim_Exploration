"""
PostgreSQL command group for MNSTEVV CLI

Handles dedicated PostgreSQL database management operations.
"""
import click
import sys
from pathlib import Path

# Add core modules to path
core_path = str(Path(__file__).parent.parent / 'core')
if core_path not in sys.path:
    sys.path.insert(0, core_path)

try:
    from service_selector import ServiceSelector
    from docker_compose import DockerComposeWrapper
except ImportError as e:
    click.echo(f"ERROR: Failed to import core modules: {e}")
    click.echo("Make sure the mnstevv package is properly installed.")
    sys.exit(1)

@click.group('postgres')
@click.pass_context
def postgres_command(ctx):
    """
    PostgreSQL database management commands.
    
    Manage PostgreSQL services independently from drone operations.
    Useful for database-only deployments, backups, and maintenance.
    
    \b
    Examples:
        mnstevv postgres up                    # Start PostgreSQL services
        mnstevv postgres down --preserve-data  # Stop while preserving data
        mnstevv postgres status                # Check database health
        mnstevv postgres backup                # Trigger database backup
        mnstevv postgres logs --follow         # View live database logs
    """
    # Ensure the context object exists
    ctx.ensure_object(dict)

@postgres_command.command('up')
@click.option('--build', is_flag=True, default=False,
              help='Force rebuild of PostgreSQL images')
@click.option('--dry-run', is_flag=True, default=False,
              help='Show commands without executing')
@click.option('--detached/--no-detached', default=True,
              help='Run in detached mode [default: detached]')
@click.option('--follow-logs', is_flag=True, default=False,
              help='Follow logs after startup')
@click.pass_context
def postgres_up(ctx, build, dry_run, detached, follow_logs):
    """
    Start PostgreSQL services only.
    
    Starts the complete PostgreSQL stack including:
    - PostgreSQL database with TimescaleDB
    - pgAdmin for administration
    - Backup service for automated backups
    """
    try:
        # Get PostgreSQL services
        services = ServiceSelector.get_postgres_services_only()
        
        # PostgreSQL specific environment variables (passwords must be set externally)
        import os
        env_vars = {
            'POSTGRES_DB': 'drone_statistics',
            'POSTGRES_USER': 'airsim_user',
            'POSTGRES_PASSWORD': os.getenv('POSTGRES_PASSWORD', ''),
            'PGADMIN_EMAIL': 'admin@example.com',
            'PGADMIN_PASSWORD': os.getenv('PGADMIN_PASSWORD', '')
        }

        # Validate required passwords are set
        if not env_vars['POSTGRES_PASSWORD']:
            click.echo("ERROR: POSTGRES_PASSWORD environment variable is required", err=True)
            click.echo("Set it with: export POSTGRES_PASSWORD='your_secure_password'", err=True)
            sys.exit(1)
        if not env_vars['PGADMIN_PASSWORD']:
            click.echo("ERROR: PGADMIN_PASSWORD environment variable is required", err=True)
            click.echo("Set it with: export PGADMIN_PASSWORD='your_secure_password'", err=True)
            sys.exit(1)
        
        click.echo(f"MNSTEVV - Starting PostgreSQL services")
        click.echo(f"Services: {', '.join(services)}")
        
        if dry_run:
            compose_file = ctx.obj['compose_file']
            click.echo(f"\nDry Run - Would execute:")
            
            cmd_parts = [
                'docker-compose',
                '-f', str(compose_file),
                '--profile', 'stats-only'
            ]
            
            # Add environment variables
            for key, value in env_vars.items():
                cmd_parts.extend(['-e', f'{key}={value}'])
            
            cmd_parts.append('up')
            if detached:
                cmd_parts.append('-d')
            if build:
                cmd_parts.append('--build')
            cmd_parts.append('--remove-orphans')
            cmd_parts.extend(services)
            
            click.echo(' '.join(cmd_parts))
            return
        
        # Initialize Docker Compose wrapper
        compose_file = ctx.obj['compose_file']
        docker_compose = DockerComposeWrapper(compose_file)
        
        # Validate compose file
        is_valid, error = docker_compose.validate_compose_file()
        if not is_valid:
            click.echo(f"Docker Compose file validation failed: {error}", err=True)
            sys.exit(1)
        
        # Start PostgreSQL services
        click.echo(f"\nStarting PostgreSQL services...")
        
        result = docker_compose.up(
            services=services,
            profile='stats-only',
            env_vars=env_vars,
            detached=detached,
            build=build,
            remove_orphans=True
        )
        
        if result.returncode == 0:
            click.echo(f"PostgreSQL services started successfully!")
            
            # Show running services
            running_services = docker_compose.get_running_services(profile='stats-only')
            if running_services:
                click.echo(f"\nRunning PostgreSQL Services:")
                for service in running_services:
                    name = service.get('Name', 'unknown')
                    status = service.get('State', 'unknown')
                    ports = service.get('Ports', '')
                    click.echo(f"   • {name}: {status} {ports}")
            
            click.echo(f"\nDatabase Access:")
            click.echo(f"   • PostgreSQL: localhost:5432 (user: airsim_user)")
            click.echo(f"   • pgAdmin: http://localhost:8080 (admin@example.com)")
            
            # Follow logs if requested
            if follow_logs:
                click.echo(f"\n📜 Following PostgreSQL logs (Ctrl+C to stop)...")
                try:
                    docker_compose.logs(
                        services=services,
                        profile='stats-only',
                        follow=True,
                        timestamps=True
                    )
                except KeyboardInterrupt:
                    click.echo(f"\nStopped following logs")
                    
        else:
            click.echo(f"PostgreSQL startup failed with exit code {result.returncode}", err=True)
            sys.exit(result.returncode)
            
    except Exception as e:
        click.echo(f"Error starting PostgreSQL services: {e}", err=True)
        sys.exit(1)

@postgres_command.command('down')
@click.option('--preserve-data', is_flag=True, default=True,
              help='Preserve database data [default: true]')
@click.option('--force', is_flag=True, default=False,
              help='Force removal without confirmation')
@click.option('--dry-run', is_flag=True, default=False,
              help='Show commands without executing')
@click.pass_context
def postgres_down(ctx, preserve_data, force, dry_run):
    """
    Stop PostgreSQL services gracefully.
    
    By default, preserves all database data. Use --no-preserve-data
    to remove database volumes (PERMANENT DATA LOSS).
    """
    try:
        compose_file = ctx.obj['compose_file']
        
        click.echo(f"MNSTEVV - Stopping PostgreSQL services")
        
        # Data preservation warning
        if not preserve_data:
            click.echo("WARNING: Database data will be PERMANENTLY DELETED!")
            if not force and not click.confirm("Are you sure you want to continue?"):
                click.echo("Operation cancelled")
                return
        else:
            click.echo("Database data will be preserved")
        
        if dry_run:
            click.echo(f"\nDry Run - Would execute:")
            
            cmd_parts = [
                'docker-compose',
                '-f', str(compose_file),
                '--profile', 'stats-only',
                'down'
            ]
            
            if not preserve_data:
                cmd_parts.append('-v')
            cmd_parts.append('--remove-orphans')
            
            click.echo(' '.join(cmd_parts))
            return
        
        # Initialize Docker Compose wrapper
        docker_compose = DockerComposeWrapper(compose_file)
        
        # Stop PostgreSQL services
        click.echo(f"\nStopping PostgreSQL services...")
        
        result = docker_compose.down(
            profile='stats-only',
            remove_volumes=not preserve_data,
            remove_orphans=True
        )
        
        if result.returncode == 0:
            click.echo(f"PostgreSQL services stopped successfully!")
            if preserve_data:
                click.echo(f"Database data preserved in volumes")
        else:
            click.echo(f"ERROR: PostgreSQL shutdown failed with exit code {result.returncode}", err=True)
            sys.exit(result.returncode)
            
    except Exception as e:
        click.echo(f"ERROR: Error stopping PostgreSQL services: {e}", err=True)
        sys.exit(1)

@postgres_command.command('status')
@click.pass_context
def postgres_status(ctx):
    """
    Check PostgreSQL service status and connectivity.
    
    Shows the health of all PostgreSQL services and tests
    database connectivity.
    """
    try:
        compose_file = ctx.obj['compose_file']
        docker_compose = DockerComposeWrapper(compose_file)
        
        click.echo(f"MNSTEVV - PostgreSQL Status Check")
        
        # Get running services
        running_services = docker_compose.get_running_services(profile='stats-only')
        
        if not running_services:
            click.echo("❌ No PostgreSQL services are running")
            click.echo("\nTo start PostgreSQL services:")
            click.echo("   mnstevv postgres up")
            return
        
        click.echo(f"\n✅ PostgreSQL Services Status:")
        for service in running_services:
            name = service.get('Name', 'unknown')
            status = service.get('State', 'unknown')
            health = service.get('Health', 'unknown')
            ports = service.get('Ports', '')
            
            status_icon = "✅" if status == "running" else "❌"
            health_icon = "🟢" if health == "healthy" else "🟡" if health == "starting" else "🔴"
            
            click.echo(f"   {status_icon} {name}: {status}")
            if health != 'unknown':
                click.echo(f"      {health_icon} Health: {health}")
            if ports:
                click.echo(f"      🔌 Ports: {ports}")
        
        # Test database connectivity
        click.echo(f"\n🔍 Testing Database Connectivity...")
        
        try:
            # Import health check utilities
            import sys
            import os
            health_utils_path = os.path.join(ctx.obj['docker_dir'].parent, 'ros2', 'src', 'airsim_ros_pkgs', 'scripts')
            if health_utils_path not in sys.path:
                sys.path.insert(0, health_utils_path)
            
            from health_utils import DatabaseHealthChecker, HealthStatus
            
            # Perform comprehensive health check
            checker = DatabaseHealthChecker()
            health_report = checker.comprehensive_health_check()
            
            # Display results with color coding
            status_icons = {
                HealthStatus.HEALTHY: "🟢",
                HealthStatus.WARNING: "🟡", 
                HealthStatus.ERROR: "🔴",
                HealthStatus.UNKNOWN: "⚪"
            }
            
            overall_icon = status_icons.get(health_report.overall_status, "⚪")
            click.echo(f"   {overall_icon} Overall Database Health: {health_report.overall_status.value.upper()}")
            
            # Connection details
            conn_icon = status_icons.get(health_report.connection_status.status, "⚪")
            click.echo(f"   {conn_icon} Connection: {health_report.connection_status.message}")
            
            # Schema details
            schema_icon = status_icons.get(health_report.schema_status.status, "⚪")
            click.echo(f"   {schema_icon} Schema: {health_report.schema_status.message}")
            
            # Performance metrics
            if health_report.performance_metrics:
                click.echo(f"   📊 Performance Metrics:")
                for metric in health_report.performance_metrics:
                    metric_icon = status_icons.get(metric.status, "⚪")
                    click.echo(f"      {metric_icon} {metric.name}: {metric.value}")
            
            # TimescaleDB metrics
            if health_report.timescaledb_metrics:
                click.echo(f"   ⏱️  TimescaleDB Status:")
                for metric in health_report.timescaledb_metrics:
                    metric_icon = status_icons.get(metric.status, "⚪")
                    click.echo(f"      {metric_icon} {metric.name}: {metric.value}")
            
        except ImportError as e:
            click.echo(f"   ⚠️  Health check utilities not available: {e}")
            click.echo(f"   📊 Basic connectivity test would be implemented here")
        except Exception as e:
            click.echo(f"   ❌ Health check failed: {e}")
            click.echo(f"   💡 Ensure PostgreSQL container is running and accessible")
        
        click.echo(f"\n🔗 Access Information:")
        click.echo(f"   • PostgreSQL: localhost:5432")
        click.echo(f"   • pgAdmin: http://localhost:8080")
        
    except Exception as e:
        click.echo(f"ERROR: Error checking PostgreSQL status: {e}", err=True)
        sys.exit(1)

@postgres_command.command('backup')
@click.option('--format', type=click.Choice(['sql', 'custom', 'directory']), default='custom',
              help='Backup format [default: custom]')
@click.option('--output', default=None,
              help='Backup output path (default: timestamp-based)')
@click.pass_context
def postgres_backup(ctx, format, output):
    """
    Trigger a manual database backup.
    
    Creates a backup of the drone statistics database using pg_dump.
    """
    try:
        click.echo(f"MNSTEVV - PostgreSQL Backup")
        click.echo(f"Format: {format}")
        
        if output:
            click.echo(f"Output: {output}")
        else:
            click.echo(f"Output: Auto-generated timestamp-based filename")
        
        # This would trigger the backup container or run pg_dump directly
        click.echo(f"\n📦 Backup operation would be implemented here")
        click.echo(f"   Would execute backup script in postgres-backup container")
        click.echo(f"   Or run pg_dump directly against the database")
        
    except Exception as e:
        click.echo(f"ERROR: Error creating backup: {e}", err=True)
        sys.exit(1)

@postgres_command.command('logs')
@click.option('--follow', is_flag=True, default=False,
              help='Follow log output')
@click.option('--service', type=click.Choice(['postgres', 'pgadmin', 'backup', 'all']), default='all',
              help='Which service logs to show [default: all]')
@click.option('--tail', type=int, default=100,
              help='Number of lines to show from end of logs [default: 100]')
@click.pass_context
def postgres_logs(ctx, follow, service, tail):
    """
    View PostgreSQL service logs.
    
    Shows logs from PostgreSQL services with filtering options.
    """
    try:
        compose_file = ctx.obj['compose_file']
        docker_compose = DockerComposeWrapper(compose_file)
        
        # Map service selection to actual service names
        if service == 'postgres':
            services = ['postgres-stats']
        elif service == 'pgadmin':
            services = ['pgadmin-stats']
        elif service == 'backup':
            services = ['postgres-backup']
        else:  # all
            services = ServiceSelector.get_postgres_services_only()
        
        click.echo(f"MNSTEVV - PostgreSQL Logs ({service})")
        
        if follow:
            click.echo(f"📜 Following logs (Ctrl+C to stop)...")
        else:
            click.echo(f"📜 Showing last {tail} lines...")
        
        try:
            docker_compose.logs(
                services=services,
                profile='stats-only',
                follow=follow,
                timestamps=True,
                tail=tail
            )
        except KeyboardInterrupt:
            if follow:
                click.echo(f"\nStopped following logs")
            
    except Exception as e:
        click.echo(f"ERROR: Error viewing logs: {e}", err=True)
        sys.exit(1)