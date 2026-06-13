# gazebo_tutorial
<img width="960" height="1280" alt="WhatsApp Image 2026-06-12 at 4 12 13 PM (1)" src="https://github.com/user-attachments/assets/1655dba0-5b2d-4db9-8d3e-4218a609b465" />





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
