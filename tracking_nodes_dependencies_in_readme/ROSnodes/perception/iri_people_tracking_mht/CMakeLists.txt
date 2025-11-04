cmake_minimum_required(VERSION 3.8)
project(iri_people_tracking_mht)

# Set C++ standard
# set(CMAKE_CXX_STANDARD 14)
# set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_ros REQUIRED)
find_package(iri_base_algorithm REQUIRED)
find_package(iri_perception_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(iriutils REQUIRED)
#find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(people_tracking_mht REQUIRED)
find_package(pluginlib REQUIRED)

# Dynamic reconfigure is not supported in ROS2. You'll need to replace it with ROS2 parameters
# Example: https://index.ros.org/doc/ros2/Tutorials/Parameters/Understanding-ROS2-Parameters/

###########
## Build ##
###########

# Include directories
include_directories(
  include
  ${people_tracking_mht_INCLUDE_DIRS}
  ${iri_base_algorithm_INCLUDE_DIRS}
  ${iri_perception_msgs_INCLUDE_DIRS}
  ${iriutils_INCLUDE_DIRS}
  #${rclcpp_INCLUDE_DIRS}
  ${nav_msgs_INCLUDE_DIRS}
  ${visualization_msgs_INCLUDE_DIRS}
  #${tf2_INCLUDE_DIRS}
  #${tf2_ros_INCLUDE_DIRS}
)

# Declare an executable
add_executable(${PROJECT_NAME} src/people_tracking_mht_alg_node.cpp src/people_tracking_mht_alg.cpp)

# Link libraries
#target_link_libraries(${PROJECT_NAME}
#  ${rclcpp_LIBRARIES}
#  ${nav_msgs_LIBRARIES}
#  ${visualization_msgs_LIBRARIES}
#  ${tf2_LIBRARIES}
#  ${tf2_ros_LIBRARIES}
#  ${iriutils_LIBRARIES}
#  ${people_tracking_mht_LIBRARIES}
#)

# Add dependencies for message headers
ament_target_dependencies(${PROJECT_NAME}
  #rclcpp
  nav_msgs
  visualization_msgs
  iri_perception_msgs
  iri_base_algorithm
  tf2
  tf2_ros
  tf2_geometry_msgs
  iriutils
  people_tracking_mht
  pluginlib
)

# Export package dependencies
#ament_export_dependencies(
#  rclcpp
#  nav_msgs
#  visualization_msgs
#  iri_perception_msgs
#  iri_base_algorithm
#  tf2
#  tf2_ros
#  iriutils
#  people_tracking_mht
#)

# Install
install(TARGETS ${PROJECT_NAME}
  DESTINATION lib/${PROJECT_NAME})

# Install launch files, if any
install(DIRECTORY launch rviz
  DESTINATION share/${PROJECT_NAME})

# Package configuration
ament_package()
