from glob import glob
from setuptools import find_packages, setup

package_name = 'gazebo_tutorial'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/worlds', glob('worlds/*.world')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='student',
    maintainer_email='student@example.com',
    description='5x5 Gazebo tutorial world with lidar robot, obstacles, and bonuses.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'collector_controller = gazebo_tutorial.collector_controller:main',
            'bonus_randomizer = gazebo_tutorial.bonus_randomizer:main',
        ],
    },
)
