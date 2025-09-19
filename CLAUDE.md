# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview
This is `btrajSim_2025-08`, a ROS-based UAV trajectory planning simulation framework called Btraj. It generates safe, dynamically feasible trajectories in unknown environments using a two-stage approach: front-end path finding (Fast Marching* or A*) and back-end trajectory optimization with Bezier curves.

## Build System & Dependencies
- **ROS Framework**: ROS Melodic (Ubuntu 18.04) - updated from Kinetic
- **Build System**: CMake with `catkin_make`
- **Key Dependencies**:
  - libarmadillo-dev (linear algebra library)
  - PCL (Point Cloud Library)
  - Eigen3
  - Mosek (QP solver - requires academic license)
  - fast_methods (Fast Marching implementation)
  - sdf_tools (Euclidean distance field computation)

## Development Commands

### Docker Environment (Recommended)
```bash
# Build and start ROS Melodic container
./build.sh

# Enter container for development
./start.sh

# Inside container - build workspace
cd /root/catkin_ws
catkin_make
source devel/setup.bash

# For development with file permissions sync:
export UID=$(id -u)
export GID=$(id -g)
# Then run build.sh
```

### Native Build
```bash
# Install dependencies
sudo apt-get install libarmadillo-dev

# Build the workspace (from catkin workspace root)
catkin_make
source devel/setup.bash
```

### Running Simulation
```bash
# Launch main simulation (includes rviz visualization)
roslaunch bezier_planer simulation.launch

# Use 3D Nav Goal tool in rviz to set waypoints:
# - Click tool, press left mouse button on position
# - Drag up/down while holding left button for height
# - Release to send target
```

### Configuration
- Set path finding algorithm in launch file: `is_use_fm` (true=Fast Marching*, false=A*)
- Map parameters, velocity/acceleration limits configurable in `simulation.launch`

## Architecture

### Core Modules
- **b_traj_node** (`Btraj/src/b_traj_node.cpp`): Main planning node integrating all components
- **TrajectoryGenerator** (`include/trajectory_generator.h`): Bezier curve trajectory optimization using Mosek QP solver
- **Bernstein** (`include/bezier_base.h`): Mathematical support for Bernstein basis polynomials and control point mapping
- **gridPathFinder** (`include/a_star.h`): A* pathfinding on grid maps
- **Fast Marching** (`third_party/fast_methods/`): FM* algorithm for path finding on velocity fields

### Key Components
- **Front-end**: Path generation using either FM* on velocity field or A* on grid map
- **Back-end**: Bezier curve trajectory optimization within flight corridors
- **Simulation Environment**: Random forest obstacle generation and sensing simulation
- **Visualization**: RViz integration with custom 3D goal tool

### Package Structure
- `catkin_ws/src/Btraj/`: Main trajectory planning package
- `catkin_ws/src/plan_utils/`: Supporting ROS packages (visualization, messages, utilities)
- `.devcontainer/`: Docker setup for ROS Melodic environment
- `mosek/`: Directory for Mosek academic license file

### Mosek License Requirement
Academic license required from mosek.com - place `mosek.lic` in `~/mosek/` directory or mount as volume in Docker setup.

## Paper Reference
This framework implements the algorithm from:
"Online Safe Trajectory Generation For Quadrotors Using Fast Marching Method and Bernstein Basis Polynomial," 
Fei Gao, William Wu, Yi Lin and Shaojie Shen, ICRA 2018.

## Key Files
- `catkin_ws/src/Btraj/launch/simulation.launch`: Main simulation configuration
- `catkin_ws/src/Btraj/CMakeLists.txt`: Build configuration and dependencies
- `.devcontainer/docker-compose.yml`: Container setup with X11 forwarding
- `catkin_ws/src/plan_utils/`: Contains quadrotor_msgs, visualization tools, and RViz plugins
- `build.sh` / `start.sh`: Docker container management scripts

## Node Architecture
- **Trajectory Planner** (`b_traj_node`): Main planning and coordination
- **Trajectory Server** (`traj_server`): Trajectory execution and monitoring
- **Odometry Generator** (`odom_generator`): Simulated robot state
- **Random Forest Sensing** (`random_forest_sensing`): Environment simulation
- **Visualization** (`vis_node`): RViz trajectory and corridor display

## Development Troubleshooting

### X11 Display Issues
```bash
# If RViz fails to start, check X11 forwarding:
echo $DISPLAY
xhost +local:docker  # On host machine
```

### Container Development
```bash
# Enter running container:
docker exec -it btraj-sim-melodic bash

# Rebuild after code changes:
cd /root/catkin_ws && catkin_make
source devel/setup.bash
```

### Parameter Tuning
- **Algorithm Selection**: `is_use_fm` in launch file (Fast Marching* vs A*)
- **Map Bounds**: `map_size_x/y/z` parameters (default: 50x50x5m)
- **Trajectory Limits**: `vel_max`, `acc_max` (default: 2.0 m/s, 2.0 m/s²)
- **Polynomial Order**: `traj_order` (default: 8th order Bezier curves)