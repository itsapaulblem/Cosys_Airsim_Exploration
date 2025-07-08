from setuptools import setup, find_packages
import os
from glob import glob

package_name = 'airsim_grid_search'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Cosys-Lab',
    maintainer_email='developer@cosys-lab.org',
    description='Grid search mission execution system for AirSim drones',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'grid_search_server = airsim_grid_search.grid_search_server:main',
            'grid_generator = airsim_grid_search.grid_generator:main',
            'mission_monitor = airsim_grid_search.mission_monitor:main',
            'grid_search_cli = airsim_grid_search.grid_search_cli:main',
        ],
    },
)