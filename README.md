# Ros2 navigation

**Short description**

This second assignment of the experimental course aims to navigate a mobile robot through a given world using the `ros2_navigation` package. The world contains four markers, each associated with a waypoint that the robot should visit to successfully detect the corresponding marker.

The waypoints position are:
1. x1 = -6.0, y1= -6.0 **due to detection marker error, we changed to** $\Longrightarrow$ x1 = -6.8, y1= -8.0
2. x2 = -6.0, y2 = 6.0;
3. x3= 6.0, y3 = -6.0; **due to detection marker error, we changed to** $\Longrightarrow$ x1 = -6.5, y1= -8.0
4. x4 = 6.0, y4 = 6.0

The assignment is divided into two main parts:

1. Move the robot to each waypoint to detect and store all marker IDs.
2. Starting from the marker with the lowest ID, navigate to the corresponding waypoint, center the marker in the camera view, take a picture, and publish the image with the circled marker on a custom topic. Repeat this process for each remaining marker in order of ascending ID.

The robot's behavior and action sequence are generated using a PDDL model, where PlanSys2 computes the plan and orchestrates the sequence of actions through the ROS 2 nodes created for this assignment.

## Setup

Run these commands from a terminal. They create a workspace, clone this repo and the external repos, copy the needed files into the external packages and update the launch files as requested.

```bash
# 1) create workspace
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src

# 2) clone repositories (clone into src so they build together)
# main repo
git clone https://github.com/AlessandroMangili/assignment2_exp.git

# external packages
git clone https://github.com/AlessandroMangili/assignment2_exp_service.git   # marker_service_pkg

git clone https://github.com/your_link_here/bme_gazebo_sensors.git           # bme_gazebo_sensors (replace with real link)

git clone https://github.com/your_link_here/aruco_opencv.git                 # aruco_opencv (replace with real link)

git clone https://github.com/CarmineD8/ros2_navigation.git                   # ros2_navigation

git clone https://github.com/CarmineD8/erl1.git                              # erl1
```

### Copy RViz configuration into `bme_gazebo_sensors`

The `bme_gazebo_sensors` package must contain the RViz config that lives in this repo (`assignment2_exp/rviz/assignment.rviz`). Create `rviz/` inside the package (if not present) and copy the file:

```bash
# from ~/ros2_ws/src
cp assignment2_exp/rviz/assignment.rviz bme_gazebo_sensors/rviz/assignment.rviz
```

Next, edit the `spawn_robot.launch.py` file in `bme_gazebo_sensors` to set the `rviz_config_arg` parameter to `assignment.rviz` and to set the `world_arg` parameter to the world filename you want to use (found inside `assignment2_exp/worlds/`). 

### Add world to `erl1` package

The `erl1` package should contain the world file present in this repo's `worlds/` folder. Copy the selected world file into `erl1/worlds/` (create the folder if needed):

```bash
mkdir -p erl1/worlds
cp assignment2_exp/worlds/<your_chosen_world>.world erl1/worlds/
```

Replace `<your_chosen_world>.world` with the actual filename you want to use.

### Add map files to `ros2_navigation`

Copy the map files (both YAML and PGM) into the `ros2_navigation/maps` folder. Create the folder if it doesn't exist.

```bash
mkdir -p ros2_navigation/maps
cp assignment2_exp/maps/map_assignment.yaml ros2_navigation/maps/
cp assignment2_exp/maps/map_assignment.png ros2_navigation/maps/
```

Now edit `ros2_navigation/launch/localization.launch.py` and change the parameter `map_file_path` to `map_assignment.yaml`.

## Build the workspace

After all repos are in `~/ros2_ws/src` and file edits/copies are complete, build the workspace.

## How to run

To run the simulation, simply type this command after building the workspace:

```bash
ros2 launch assignment2_exp start_assignment.launch.py
```

This will open two additional terminals: one, labeled with the Executors name, runs the ROS 2 nodes, instead the other, labeled Plan, runs the ```get_plan_and_execute``` node.

**Be careful**: to use this launch file, ```gnome-terminal``` must be installed.
