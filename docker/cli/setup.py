#!/usr/bin/env python3
"""
Setup configuration for MNSTEVV CLI tool.
"""
from setuptools import setup, find_packages
from pathlib import Path

# Read the README file for long description
this_directory = Path(__file__).parent
try:
    long_description = (this_directory / "README.md").read_text(encoding='utf-8') if (this_directory / "README.md").exists() else """
MNSTEVV - Multi-Node System Test Environment Vehicle Validator

A CLI tool for managing AirSim Docker Compose ecosystems with intelligent
service selection based on drone count and deployment profiles.

Features:
- Intelligent service selection based on --num_drones parameter
- Profile-based deployments (integrated, px4-only, ros2-only, development)
- Built-in health monitoring and status checks
- Configuration profile management
- Comprehensive logging and debugging capabilities
"""
except UnicodeDecodeError:
    long_description = """
MNSTEVV - Multi-Node System Test Environment Vehicle Validator

A CLI tool for managing AirSim Docker Compose ecosystems with intelligent
service selection based on drone count and deployment profiles.

Features:
- Intelligent service selection based on --num_drones parameter
- Profile-based deployments (integrated, px4-only, ros2-only, development)
- Built-in health monitoring and status checks
- Configuration profile management
- Comprehensive logging and debugging capabilities
"""

setup(
    name="mnstevv",
    version="1.0.1",
    author="DSTA M&S TEVV Team",
    description="Multi-Node System Test Environment Vehicle Validator - CLI for AirSim Docker ecosystems",
    long_description=long_description,
    long_description_content_type="text/markdown",
    packages=find_packages(),
    include_package_data=True,
    python_requires=">=3.7",
    install_requires=[
        "click>=8.0.0",
        "PyYAML>=5.4.0",
        "rich>=10.0.0",
    ],
    extras_require={
        "dev": [
            "pytest>=6.0",
            "pytest-cov>=2.0",
            "black>=21.0",
            "flake8>=3.9",
        ]
    },
    entry_points={
        "console_scripts": [
            "mnstevv=mnstevv.main:cli",
        ],
    },
    classifiers=[
        "Development Status :: 4 - Beta",
        "Intended Audience :: Developers",
        "License :: OSI Approved :: MIT License",
        "Operating System :: OS Independent",
        "Programming Language :: Python :: 3",
        "Programming Language :: Python :: 3.7",
        "Programming Language :: Python :: 3.8",
        "Programming Language :: Python :: 3.9",
        "Programming Language :: Python :: 3.10",
        "Programming Language :: Python :: 3.11",
        "Topic :: Software Development :: Build Tools",
        "Topic :: System :: Systems Administration",
        "Topic :: Scientific/Engineering :: Interface Engine/Protocol Translator",
    ],
    keywords="airsim docker ros2 px4 drone simulation cli",
    project_urls={
        "Bug Reports": "https://github.com/cosys-lab/airsim-issues",
        "Source": "https://github.com/microsoft/airsim",
        "Documentation": "https://microsoft.github.io/AirSim/",
    },
)