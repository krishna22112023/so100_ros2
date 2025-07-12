####################################################################################################
#                                         PROJECT INFORMATION                                      #
####################################################################################################
cmake_minimum_required(VERSION 3.5)
project(so100_ros_controller)

####################################################################################################
#                                          DEPENDENCIES                                            #
####################################################################################################
find_package(ament_cmake REQUIRED)
find_package(glfw3 REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)                                                                 

# MuJoCo
set(MUJOCO_PATH "/home/krishna/Workspace/mujoco-3.3.4")
include_directories(include)                                                                        
include_directories(${MUJOCO_PATH}/include/mujoco)
include_directories(${MUJOCO_PATH}/include)
link_directories(${MUJOCO_PATH}/lib)

####################################################################################################
#                                           EXECUTABLES                                            #
####################################################################################################
# position controller node
add_executable(position_commander src/motion_position_control/position_commander.cpp)
ament_target_dependencies(position_commander
  rclcpp
  std_msgs
)

# mujoco ros node
add_executable(mujoco_ros src/motion_position_control/mujoco_ros.cpp)
target_link_libraries(mujoco_ros mujoco glfw) 
ament_target_dependencies(mujoco_ros rclcpp sensor_msgs std_msgs)  

####################################################################################################
#                                           INSTALLATION                                           #
####################################################################################################

install(TARGETS position_commander mujoco_ros
        DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch model
        DESTINATION share/${PROJECT_NAME})

  ament_package()