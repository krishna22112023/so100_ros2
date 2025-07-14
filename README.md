# ROS2-SO100 controller with MuJoCo simulation
This package implements a basic MuJoCo simulation for sending position/velocity/torque commands for open loop motion control of the LeRobot SO100 6DOF arm.


## Dependencies

- [Ubuntu 22.04](https://ubuntu.com/download/desktop) or later,
- [ROS2](https://docs.ros.org/en/humble/index.html) (Humble Hawksbill)
- [MuJoCo 3.3.4](https://mujoco.org/) or later, and
- [GLFW](https://github.com/glfw/glfw) for visualisation.

## Install

1. **Install ROS2:**
   
   Follow the [ROS2 installation guide](https://docs.ros.org/en/foxy/Installation.html) for your operating system.

2. **Install MuJoCo:**
   
   Download and install MuJoCo from the [official website](https://mujoco.org/).

   Note : pip will not work. Please install from source.

   Mine is installed in `root_path/mujoco-3.3.4`

   Next, modify your `.bashrc` file:
   ```
   gedit ~/.bashrc
   ```
   Add the following lines of code at the bottom:
   ```
   export MUJOCO_PY_MUJOCO_PATH=root_path/mujoco-3.3.4
   export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:root_path/mujoco-3.3.4/bin
   export LD_LIBRARY_PATH=root_path/mujoco-3.3.4/lib:$LD_LIBRARY_PATH
   ```
   Change all instances of `root_path/mujoco-3.3.4` to match your installation location, and your version.

4. **Install GLFW:**
```
   sudo apt-get install libglfw3-dev
```

## Build

clone the repository: 
```
https://github.com/krishna22112023/so100_ros2.git
```


In `CMakeLists.txt` you need to tell the compiler where to find the MuJoCo header files:
```
set(MUJOCO_PATH "root_path/mujoco-3.3.4") # UPDATE THIS TO YOUR MUJOCO PATH
include_directories(${MUJOCO_PATH}/include/mujoco)                                                  # MuJoCo header files
include_directories(${MUJOCO_PATH}/include)                                                         # Additional MuJoCo header files
link_directories(${MUJOCO_PATH}/lib)                                                                # Location of MuJoCo libraries
```

Build the packages
```
colcon build --symlink-install
```
You should now see `build`,`install` and a `log` folders.

Source the ROS2 workspace:
```
source install/setup.bash
```

## Run position control

To launch the mujoco_ros node and the position_commander node, along with the target positions (in radians) as arguments
```
ros2 launch so100_ros_controller position_control_open_loop.py target_positions:="[0.5,-0.3,0.0,0.0,0.0,0.8]"
```
The mujoco_ros node subscribes to the `position_command` topic published by the position_commander node.
While the mujoco_ros node published to the `joint_state` topic. 
