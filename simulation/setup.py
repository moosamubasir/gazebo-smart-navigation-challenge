from setuptools import setup


package_name = "gazebo_tutorial"

setup(
    name=package_name,
    version="0.0.1",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
        (f"share/{package_name}/launch", ["launch/gazebo_tutorials.launch.py"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="afeez",
    maintainer_email="afeez@example.com",
    description="5x5 grid robot ROS2 package",
    license="MIT",
    entry_points={
        "console_scripts": [
            
            "bonus_randomizer = gazebo_tutorial.bonus_randomizer:main",
            "grid_navigator = gazebo_tutorial.grid_navigator:main",
            "collector_controller = gazebo_tutorial.collector_controller:main",
            
        ],
    },
)
