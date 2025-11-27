"""
Config command implementation for MNSTEVV CLI

Manages configuration profiles for frequently used setups.
"""
import click
import sys
import os
import yaml
from pathlib import Path

# Configuration directory
CONFIG_DIR = Path.home() / '.mnstevv'
CONFIG_FILE = CONFIG_DIR / 'config.yaml'

def ensure_config_dir():
    """Ensure configuration directory exists."""
    CONFIG_DIR.mkdir(exist_ok=True)

def load_config():
    """Load configuration file."""
    if not CONFIG_FILE.exists():
        return {}
    try:
        with open(CONFIG_FILE, 'r') as f:
            return yaml.safe_load(f) or {}
    except Exception:
        return {}

def save_config(config):
    """Save configuration file."""
    ensure_config_dir()
    try:
        with open(CONFIG_FILE, 'w') as f:
            yaml.dump(config, f, default_flow_style=False, sort_keys=True)
        return True
    except Exception as e:
        click.echo(f"ERROR: Error saving config: {e}", err=True)
        return False

@click.group('config')
def config_command():
    """
    Manage configuration profiles for frequently used setups.
    
    Configuration profiles allow you to save and reuse common drone swarm
    configurations, making it easy to switch between different test scenarios.
    """
    pass

@config_command.command('save')
@click.argument('name')
@click.option('--num_drones', type=click.IntRange(1, 6), required=True,
              help='Number of drones for this profile')
@click.option('--profile', '-p', required=True,
              type=click.Choice(['integrated', 'px4-only', 'ros2-only', 'development']),
              help='Deployment profile')
@click.option('--debug', is_flag=True, default=False,
              help='Enable debug logging')
@click.option('--ros-domain-id', type=int, default=0,
              help='ROS domain ID')
@click.option('--airsim-host', default=None,
              help='AirSim host IP')
@click.option('--description', default=None,
              help='Description of this configuration')
def save_config_profile(name, num_drones, profile, debug, ros_domain_id, airsim_host, description):
    """
    Save a new configuration profile.
    
    \b
    Examples:
        mnstevv config save test-swarm --num_drones 3 --profile integrated
        mnstevv config save px4-test --num_drones 2 --profile px4-only --debug
        mnstevv config save dev-setup --num_drones 1 --profile development --description "Single drone dev environment"
    """
    config = load_config()
    
    profile_config = {
        'num_drones': num_drones,
        'profile': profile,
        'debug': debug,
        'ros_domain_id': ros_domain_id
    }
    
    if airsim_host:
        profile_config['airsim_host'] = airsim_host
    
    if description:
        profile_config['description'] = description
    
    # Store in profiles section
    if 'profiles' not in config:
        config['profiles'] = {}
    
    config['profiles'][name] = profile_config
    
    if save_config(config):
        click.echo(f"Saved configuration profile: {name}")
        click.echo(f"   Drones: {num_drones}")
        click.echo(f"   Profile: {profile}")
        if description:
            click.echo(f"   Description: {description}")
    else:
        sys.exit(1)

@config_command.command('load')
@click.argument('name')
def load_config_profile(name):
    """
    Show configuration for a saved profile.
    
    \b
    Examples:
        mnstevv config load test-swarm     # Show test-swarm configuration
        mnstevv config load px4-test       # Show px4-test configuration
    """
    config = load_config()
    profiles = config.get('profiles', {})
    
    if name not in profiles:
        click.echo(f"ERROR: Profile '{name}' not found", err=True)
        click.echo(f"Available profiles: {', '.join(profiles.keys()) if profiles else 'none'}")
        sys.exit(1)
    
    profile_config = profiles[name]
    
    click.echo(f"Configuration Profile: {name}")
    click.echo("=" * 40)
    click.echo(f"Drones: {profile_config['num_drones']}")
    click.echo(f"Profile: {profile_config['profile']}")
    click.echo(f"Debug: {profile_config.get('debug', False)}")
    click.echo(f"ROS Domain ID: {profile_config.get('ros_domain_id', 0)}")
    
    if profile_config.get('airsim_host'):
        click.echo(f"AirSim Host: {profile_config['airsim_host']}")
    
    if profile_config.get('description'):
        click.echo(f"Description: {profile_config['description']}")
    
    click.echo(f"\nUse with: mnstevv up --config {name}")

@config_command.command('list')
def list_config_profiles():
    """
    List all saved configuration profiles.
    
    \b
    Example:
        mnstevv config list               # Show all saved profiles
    """
    config = load_config()
    profiles = config.get('profiles', {})
    
    if not profiles:
        click.echo("No configuration profiles saved")
        click.echo("Save a profile with: mnstevv config save <name> --num_drones 3 --profile integrated")
        return
    
    click.echo(f"Saved Configuration Profiles:")
    click.echo("=" * 40)
    
    for name, profile_config in profiles.items():
        drones = profile_config['num_drones']
        profile = profile_config['profile']
        description = profile_config.get('description', '')
        
        click.echo(f"• {name}")
        click.echo(f"  {drones} drone{'s' if drones > 1 else ''}, {profile} profile")
        if description:
            click.echo(f"  {description}")
        click.echo()

@config_command.command('delete')
@click.argument('name')
@click.confirmation_option(prompt=f'Delete configuration profile?')
def delete_config_profile(name):
    """
    Delete a saved configuration profile.
    
    \b
    Examples:
        mnstevv config delete old-test     # Delete old-test profile
    """
    config = load_config()
    profiles = config.get('profiles', {})
    
    if name not in profiles:
        click.echo(f"ERROR: Profile '{name}' not found", err=True)
        sys.exit(1)
    
    del profiles[name]
    config['profiles'] = profiles
    
    if save_config(config):
        click.echo(f"Deleted configuration profile: {name}")
    else:
        sys.exit(1)

@config_command.command('path')
def show_config_path():
    """Show path to configuration file."""
    click.echo(f"Configuration file: {CONFIG_FILE}")
    click.echo(f"Configuration directory: {CONFIG_DIR}")
    
    if CONFIG_FILE.exists():
        click.echo(f"Configuration file exists")
        config = load_config()
        profiles = config.get('profiles', {})
        click.echo(f"Profiles stored: {len(profiles)}")
    else:
        click.echo(f"No configuration file (will be created when first profile is saved)")

def get_config_profile(profile_name: str) -> dict:
    """
    Get configuration profile by name. Used by other commands.
    
    Args:
        profile_name: Name of the profile to load
        
    Returns:
        Dictionary with profile configuration
        
    Raises:
        ValueError: If profile doesn't exist
    """
    config = load_config()
    profiles = config.get('profiles', {})
    
    if profile_name not in profiles:
        available = list(profiles.keys())
        raise ValueError(f"Profile '{profile_name}' not found. Available: {', '.join(available) if available else 'none'}")
    
    return profiles[profile_name]