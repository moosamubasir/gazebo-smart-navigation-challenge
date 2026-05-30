# gazebo_tutorial

ROS 2 Gazebo Classic demo for a 5x5 arena:

- green start cell
- red end cell
- robot with differential drive and lidar
- 5 fixed obstacles
- 3 yellow bonus points that disappear when collected
- controller that visits all bonuses first, then drives to the red end cell

## Use

Put this folder inside your ROS 2 workspace `src` folder, then run:

```bash
cd ~/ros2_ws
colcon build --packages-select gazebo_tutorial
source install/setup.bash
ros2 launch gazebo_tutorial gazebo_tutorials.launch.py
```

If your workspace is on Windows through WSL, use the same commands inside Ubuntu.
