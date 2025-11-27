#!/usr/bin/env python3
"""
MNSTEVV - Multi-Node System Test Environment Vehicle Validator
A CLI tool for managing AirSim Docker Compose ecosystems.
"""
import sys
import os

# Add the mnstevv package to Python path
script_dir = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(script_dir, 'mnstevv'))

from main import cli

if __name__ == '__main__':
    cli()