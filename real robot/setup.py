from setuptools import setup, find_packages
import os
from glob import glob

package_name = 'follow_grid'

setup(
    name=package_name,
    version='2.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
            glob('launch/*.py')),
        (os.path.join('share', package_name, 'resource'),
            glob('resource/*') if os.path.exists('resource') else []),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='afeez',
    maintainer_email='afeez@tce.edu',
    description='TCE Robot — follow grid with Q-learning',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'autonomous_robot = follow_grid.autonomous_robot:main',
            'web_dashboard    = follow_grid.web_dashboard:main',
            'camera_vision    = follow_grid.camera_vision:main',
            'grid_manager     = follow_grid.grid_manager:main',
            'hardware_bridge  = follow_grid.hardware_bridge:main',
            'tune_colors      = follow_grid.tune_colors:main',
            'calibrate_warp   = follow_grid.calibrate_warp:main',
        ],
    },
)