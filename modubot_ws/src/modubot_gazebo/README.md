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
before activating Nav2. When the workspace is under `/mnt/c`, WSL may take
about one or two minutes before printing the first Gazebo process messages;
this initial silence is expected. The launch also isolates ROS discovery to
the local WSL instance so stale Jetson, VPN, or discovery-server settings do
not interfere with the simulation.

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

## Odometry-stopped trajectory experiment

The simulation backend validates the trajectory geometry, ROS command path,
odometry-based stopping, and CSV generation. It does not emulate the embedded
PI controller, battery, motor dead zone, or physical caster behavior.

Run one continuous figure eight with a 0.35 m radius:

```bash
ros2 launch modubot_gazebo trajectory_experiment_sim.launch.py
```

Parameters can be changed from the command line:

```bash
ros2 launch modubot_gazebo trajectory_experiment_sim.launch.py \
  figure_eight_radius:=0.40 figure_eight_cycles:=2 \
  linear_speed:=0.15 use_gazebo_gui:=true
```

The launch starts Gazebo, waits for `/odom` and the `/cmd_vel_safe` subscriber,
runs automatically, stores results in `~/modubot_trajectory_data`, and shuts
Gazebo down when the experiment finishes. The same runner can also target an
already running simulator:

```bash
ros2 run modubot_serial_bridge odometry_trajectory_experiment \
  --backend ros --cmd-vel-topic /cmd_vel_safe --odom-topic /odom \
  --trajectories figure_eight --repetitions 1 \
  --figure-eight-radius 0.35 --figure-eight-cycles 1 \
  --linear-speed 0.15 --automatic --inter-run-wait 0
```
