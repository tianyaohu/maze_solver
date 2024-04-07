
# ROS2 2D MazeSolver

This is a 2d maze solver. It solve simple 2D mazes by first manually drawing a map with slam_toolbox. Then it uses manually predefined waypoints to navigate the robot through the maze. The main objective is to learn to design and tune PID controllers. For future project, I will try make robot to autonomously draw map and navigate maze from p1 to p2 via pathfinding algorithm like A*, RRT, or Dijkstra. 

## 📦 Installation Instructions
- ROS2 Humble
- Rviz Version ?>??"?
- Gazebo Simulation
- slam_toolbox
- Armadillo
#### Software pacakges
- rosbot_xl_gazebo

## 📁 Build Packages
```bash
cd ~/ros2_ws
#due to interdependencies, build lib packages first, otherwise colcon complains
colcon build --packages-select pid_lib ctrl_lib; source install/setup.bash
#build everything
colcon build; source install/setup.bash
```

## Start Simulation
```bash
ros2 launch rosbot_xl_gazebo simulation.launch.py
```

## 🧭 Start Mapping
```bash
#run Teleop for key board control
cd ~/ros2_ws
source install/setup.bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

```bash
#Start Mapping
cd ~/ros2_ws
colcon build; source install/setup.bash
ros2 launch my_slam_toolbox_pkg mapping.launch.py
```

# MAPPING GIF TO BE ADDED, RVIZ, Simulation

```bash
#To Save Map
ros2 service call /slam_toolbox/save_map slam_toolbox/srv/SaveMap "{name: {data: 'your_map_name'}}"
```

## 📍 Start AMCL Localization
```bash
cd ~/ros2_ws
colcon build; source install/setup.bash
# For Simulation
ros2 launch localization sim_localization.launch.py
# For Real Robot (the Construct Cyberlab)
ros2 launch localization real_localization.launch.py
```

<strong style="color: red; font-size: 20px;">A manual pose estimation is needed</strong>

Please use the Pose Estimation tool in Rviz and localize the robot somewhat aaccurately. This initialization heavily influence whether the robot can solve maze via way_points, this approach is rather naive and brittle.

# GIF IMAGE OF POSE ESTIMATION HERE, 



<b>If using waypoint to solve maze</b>

```bash
# Print the current odom
ros2 topic echo /rosbot_xl_base_controller/odom --field pose.pose.position
```
```bash
# Print the current AMCL pose (your amcl_pose topic may be different)
ros2 topic echo /amcl_pose --field pose.pose.position
```

## 🏃‍♂️ Solving Maze
Once the your waypoints are saved as yaml files within `maze_solver\pid_maze_solver\waypoints`, run the `naive_maze_solver.launch.py` file to solve maze via waypoints. Change way_point file name within launch file, if you are using your own waypoints.
```bash
# scene:=0 is for simulation; 1 is real_robot
# reverse_solve if false, follows the yaml file order; true reverse the order
ros2 launch pid_maze_solver naive_maze_solver.launch.py scene_num:=0 reverse_solve:=false
```

<strong style="color: green;"> The robot can solve maze right in the middle of the maze. As long as it is somewhere along the solution path, the robot will try to move to the closest waypoint and solve the maze from there. FYI: this doesn't work if there is an obstable inbetween the robot and waypoint.  </strong><strong style="color: red;">This is a naive maze solver, it does NOT have any obstacle avoidance.</strong>


# Maze solve Video HERE
