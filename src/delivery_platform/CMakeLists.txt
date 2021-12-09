################################################################################
# Set minimum required version of cmake, project name and compile options
################################################################################
cmake_minimum_required(VERSION 3.0.2)
project(delivery_platform)
cmake_policy(SET CMP0054 NEW)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

################################################################################
# Find catkin packages and libraries for catkin and system dependencies
################################################################################
#set(CMAKE_PREFIX_PATH "/home/jia/Desktop/sunday/devel/share")

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  sensor_msgs
  geometry_msgs
  nav_msgs
  tf
  gazebo_ros
  message_generation
  apriltag_ros
  a_user_portal
)

find_package(gazebo REQUIRED)

################################################################################
# Setup for python modules and scripts
################################################################################

################################################################################
# Declare ROS messages, services and actions
################################################################################
add_message_files(
  FILES
  destination.msg
  delivered.msg
  delivering.msg
  preloadpos.msg
  location.msg
)
generate_messages(
  DEPENDENCIES
  std_msgs
)
################################################################################
# Declare ROS dynamic reconfigure parameters
################################################################################

################################################################################
# Declare catkin specific configuration to be passed to dependent projects
################################################################################
catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS message_runtime roscpp std_msgs sensor_msgs geometry_msgs nav_msgs tf gazebo_ros
  DEPENDS GAZEBO
)

################################################################################
# Build
################################################################################
link_directories(${GAZEBO_LIBRARY_DIRS})

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${GAZEBO_INCLUDE_DIRS}
)

add_executable(main src/main.cpp src/TurtleBot.cpp src/Camera.cpp src/Restaurant.cpp src/House.cpp src/Preloading_system.cpp)
add_dependencies(main ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(main ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})



################################################################################
# Install
################################################################################
install(TARGETS main
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(DIRECTORY launch models rviz worlds 
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

################################################################################
# Test
################################################################################
