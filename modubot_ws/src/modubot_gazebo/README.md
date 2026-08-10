# ModuBot simulation

This package keeps the Gazebo Classic simulation separate from the physical
robot launch while reusing the same robot description, Nav2 parameters,
behavior tree, scan filter, collision monitor, and DC first-floor map.

The physical launch remains unchanged:

```bash
ros2 launch modubot_nav2 nav2.launch.py
```

## Dependencies

On ROS 2 Humble, install the workspace dependencies with `rosdep` or install
Gazebo Classic integration explicitly:

```bash
sudo apt update
sudo apt install ros-humble-gazebo-ros-pkgs
```

## Build

```bash
cd ~/ufscar/modubot_ros2/modubot_ws
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
```

## Complete navigation simulation

```bash
ros2 launch modubot_nav2 nav2_sim.launch.py
```

The launch waits for Gazebo to load the detailed DC world and robot meshes
before activating Nav2. On slower computers, the first window may therefore
take about 20--30 seconds to appear.

Useful options:

```bash
# Run Gazebo and Nav2 without graphical clients.
ros2 launch modubot_nav2 nav2_sim.launch.py \
  use_gazebo_gui:=false use_rviz:=false

# Start the robot at another pose in the map/world.
ros2 launch modubot_nav2 nav2_sim.launch.py \
  x:=1.0 y:=2.0 yaw:=1.57
```

The default pose is `(0, 0, 0)` and matches the Nav2 initial pose. If another
spawn pose is used, set the same initial pose in RViz before sending a goal.

## Gazebo only

```bash
ros2 launch modubot_gazebo gazebo.launch.py
```

The simulated differential drive listens to `/cmd_vel_safe`. This preserves
the same command chain used on the real robot:

`Nav2 /cmd_vel -> Collision Monitor /cmd_vel_safe -> simulated or real base`.
