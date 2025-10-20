cmake_minimum_required(VERSION 2.8.3)
project(iri_people_tracking_mht)

SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x") 
## Find catkin macros and libraries
find_package(catkin REQUIRED)
# ******************************************************************** 
#                 Add catkin additional components here
# ******************************************************************** 
find_package(catkin REQUIRED COMPONENTS iri_base_algorithm nav_msgs visualization_msgs iri_perception_msgs tf)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)

# ******************************************************************** 
#           Add system and labrobotica dependencies here
# ******************************************************************** 
find_package(iriutils REQUIRED)
#find_package(people_prediction REQUIRED)
find_package(people_tracking_mht REQUIRED)

# ******************************************************************** 
#           Add topic, service and action definition here
# ******************************************************************** 
## Generate messages in the 'msg' folder
# add_message_files(
#   FILES
#   Message1.msg
#   Message2.msg
# )

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
# generate_messages(
#   DEPENDENCIES
#   std_msgs  # Or other packages containing msgs
# )

# ******************************************************************** 
#                 Add the dynamic reconfigure file 
# ******************************************************************** 
generate_dynamic_reconfigure_options(cfg/PeopleTrackingMht.cfg)

# ******************************************************************** 
#                 Add run time dependencies here
# ******************************************************************** 
catkin_package(
#  INCLUDE_DIRS 
#  LIBRARIES 
# ******************************************************************** 
#            Add ROS and IRI ROS run time dependencies
# ******************************************************************** 
  CATKIN_DEPENDS iri_base_algorithm nav_msgs visualization_msgs iri_perception_msgs tf
# ******************************************************************** 
#      Add system and labrobotica run time dependencies here
# ******************************************************************** 
  DEPENDS iriutils  people_tracking_mht
)

###########
## Build ;  DEPENDS  people_prediction##
###########

# ******************************************************************** 
#                   Add the include directories 
# ******************************************************************** 
include_directories(include)
include_directories(${catkin_INCLUDE_DIRS})
# include_directories(${<dependency>_INCLUDE_DIR})
include_directories(${iriutils_INCLUDE_DIR})
#include_directories(${people_prediction_INCLUDE_DIR})
include_directories(${people_tracking_mht_INCLUDE_DIR})

## Declare a cpp library
# add_library(${PROJECT_NAME} <list of source files>)

## Declare a cpp executable
add_executable(${PROJECT_NAME} src/people_tracking_mht_alg_node.cpp src/people_tracking_mht_alg.cpp)

# ******************************************************************** 
#                   Add the libraries
# ******************************************************************** 
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})
# target_link_libraries(${PROJECT_NAME} ${<dependency>_LIBRARY})
target_link_libraries(${PROJECT_NAME} ${iriutils_LIBRARY})
#target_link_libraries(${PROJECT_NAME} ${people_prediction_LIBRARY})
target_link_libraries(${PROJECT_NAME} ${people_tracking_mht_LIBRARY})

# ******************************************************************** 
#               Add message headers dependencies 
# ******************************************************************** 
# add_dependencies(${PROJECT_NAME} <msg_package_name>_generate_messages_cpp)
add_dependencies(${PROJECT_NAME} iri_perception_msgs_generate_messages_cpp)
# ******************************************************************** 
#               Add dynamic reconfigure dependencies 
# ******************************************************************** 
add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS})

