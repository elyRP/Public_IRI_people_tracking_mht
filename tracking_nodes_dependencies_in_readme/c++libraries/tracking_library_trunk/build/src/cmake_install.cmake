# Install script for directory: /home/elyupc2204msi/ros2_ws/src/codeSubjectMobileRobots/plannerSRC/tracker_node-tracker_node_in_ROS2_humble/iri_people_tracking_mht/tracking_library_trunk/src

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/usr/local")
endif()
string(REGEX REPLACE "/$" "" CMAKE_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}")

# Set the install configuration name.
if(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)
  if(BUILD_TYPE)
    string(REGEX REPLACE "^[^A-Za-z0-9_]+" ""
           CMAKE_INSTALL_CONFIG_NAME "${BUILD_TYPE}")
  else()
    set(CMAKE_INSTALL_CONFIG_NAME "DEBUG")
  endif()
  message(STATUS "Install configuration: \"${CMAKE_INSTALL_CONFIG_NAME}\"")
endif()

# Set the component getting installed.
if(NOT CMAKE_INSTALL_COMPONENT)
  if(COMPONENT)
    message(STATUS "Install component: \"${COMPONENT}\"")
    set(CMAKE_INSTALL_COMPONENT "${COMPONENT}")
  else()
    set(CMAKE_INSTALL_COMPONENT)
  endif()
endif()

# Install shared libraries without execute permission?
if(NOT DEFINED CMAKE_INSTALL_SO_NO_EXE)
  set(CMAKE_INSTALL_SO_NO_EXE "1")
endif()

# Is this installation the result of a crosscompile?
if(NOT DEFINED CMAKE_CROSSCOMPILING)
  set(CMAKE_CROSSCOMPILING "FALSE")
endif()

# Set default install directory permissions.
if(NOT DEFINED CMAKE_OBJDUMP)
  set(CMAKE_OBJDUMP "/usr/bin/objdump")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/iri/people_tracking_mht/scene_elements" TYPE FILE FILES
    "/home/elyupc2204msi/ros2_ws/src/codeSubjectMobileRobots/plannerSRC/tracker_node-tracker_node_in_ROS2_humble/iri_people_tracking_mht/tracking_library_trunk/src/../include/scene_elements/person_abstract.h"
    "/home/elyupc2204msi/ros2_ws/src/codeSubjectMobileRobots/plannerSRC/tracker_node-tracker_node_in_ROS2_humble/iri_people_tracking_mht/tracking_library_trunk/src/../include/scene_elements/person_bhmip.h"
    "/home/elyupc2204msi/ros2_ws/src/codeSubjectMobileRobots/plannerSRC/tracker_node-tracker_node_in_ROS2_humble/iri_people_tracking_mht/tracking_library_trunk/src/../include/scene_elements/robot.h"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/iri/people_tracking_mht/libpeople_tracking_mht.so" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/iri/people_tracking_mht/libpeople_tracking_mht.so")
    file(RPATH_CHECK
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/iri/people_tracking_mht/libpeople_tracking_mht.so"
         RPATH "")
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/iri/people_tracking_mht" TYPE SHARED_LIBRARY FILES "/home/elyupc2204msi/ros2_ws/src/codeSubjectMobileRobots/plannerSRC/tracker_node-tracker_node_in_ROS2_humble/iri_people_tracking_mht/tracking_library_trunk/lib/libpeople_tracking_mht.so")
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/iri/people_tracking_mht/libpeople_tracking_mht.so" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/iri/people_tracking_mht/libpeople_tracking_mht.so")
    if(CMAKE_INSTALL_DO_STRIP)
      execute_process(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/iri/people_tracking_mht/libpeople_tracking_mht.so")
    endif()
  endif()
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/iri/people_tracking_mht" TYPE FILE FILES
    "/home/elyupc2204msi/ros2_ws/src/codeSubjectMobileRobots/plannerSRC/tracker_node-tracker_node_in_ROS2_humble/iri_people_tracking_mht/tracking_library_trunk/src/../include/track.h"
    "/home/elyupc2204msi/ros2_ws/src/codeSubjectMobileRobots/plannerSRC/tracker_node-tracker_node_in_ROS2_humble/iri_people_tracking_mht/tracking_library_trunk/src/../include/mht.h"
    "/home/elyupc2204msi/ros2_ws/src/codeSubjectMobileRobots/plannerSRC/tracker_node-tracker_node_in_ROS2_humble/iri_people_tracking_mht/tracking_library_trunk/src/../include/mhtStructs.h"
    "/home/elyupc2204msi/ros2_ws/src/codeSubjectMobileRobots/plannerSRC/tracker_node-tracker_node_in_ROS2_humble/iri_people_tracking_mht/tracking_library_trunk/src/../include/Clusters.h"
    "/home/elyupc2204msi/ros2_ws/src/codeSubjectMobileRobots/plannerSRC/tracker_node-tracker_node_in_ROS2_humble/iri_people_tracking_mht/tracking_library_trunk/src/../include/odometry.h"
    "/home/elyupc2204msi/ros2_ws/src/codeSubjectMobileRobots/plannerSRC/tracker_node-tracker_node_in_ROS2_humble/iri_people_tracking_mht/tracking_library_trunk/src/../include/total_velocity.h"
    "/home/elyupc2204msi/ros2_ws/src/codeSubjectMobileRobots/plannerSRC/tracker_node-tracker_node_in_ROS2_humble/iri_people_tracking_mht/tracking_library_trunk/src/../include/Groups.h"
    "/home/elyupc2204msi/ros2_ws/src/codeSubjectMobileRobots/plannerSRC/tracker_node-tracker_node_in_ROS2_humble/iri_people_tracking_mht/tracking_library_trunk/src/../include/prediction_bhmip.h"
    "/home/elyupc2204msi/ros2_ws/src/codeSubjectMobileRobots/plannerSRC/tracker_node-tracker_node_in_ROS2_humble/iri_people_tracking_mht/tracking_library_trunk/src/../include/scene_abstract.h"
    "/home/elyupc2204msi/ros2_ws/src/codeSubjectMobileRobots/plannerSRC/tracker_node-tracker_node_in_ROS2_humble/iri_people_tracking_mht/tracking_library_trunk/src/../include/iri_geometry.h"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  list(APPEND CMAKE_ABSOLUTE_DESTINATION_FILES
   "/usr/share/cmake-3.22/Modules/Findpeople_tracking_mht.cmake")
  if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  file(INSTALL DESTINATION "/usr/share/cmake-3.22/Modules" TYPE FILE FILES "/home/elyupc2204msi/ros2_ws/src/codeSubjectMobileRobots/plannerSRC/tracker_node-tracker_node_in_ROS2_humble/iri_people_tracking_mht/tracking_library_trunk/src/../Findpeople_tracking_mht.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for each subdirectory.
  include("/home/elyupc2204msi/ros2_ws/src/codeSubjectMobileRobots/plannerSRC/tracker_node-tracker_node_in_ROS2_humble/iri_people_tracking_mht/tracking_library_trunk/build/src/examples/cmake_install.cmake")

endif()

