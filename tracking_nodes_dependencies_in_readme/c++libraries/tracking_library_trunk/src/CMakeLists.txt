 # locate the necessary dependencies


INCLUDE_DIRECTORIES(.)
find_package(Eigen QUIET)
if(Eigen_FOUND)
  MESSAGE("Eigen found in CMake modules:" ${Eigen_INCLUDE_DIRS})
ELSE(Eigen_FOUND)  
    #INCLUDE_DIRECTORIES(/usr/include/eigen3)
    #INCLUDE_DIRECTORIES(/usr/local/include/eigen3)
    MESSAGE("Eigen not found, adding manually directories. BE CAREFULL to have installed eigen there")
    SET(Eigen_INCLUDE_DIRS /usr/include/eigen3 /usr/local/include/eigen3)    
ENDIF(Eigen_FOUND)
INCLUDE_DIRECTORIES(${Eigen_INCLUDE_DIRS})

SET(sources_scene_elements
scene_elements/person_abstract.cpp scene_elements/person_bhmip.cpp scene_elements/robot.cpp
)

SET(headers_scene_elements
../include/scene_elements/person_abstract.h ../include/scene_elements/person_bhmip.h ../include/scene_elements/robot.h
)

# driver source files
SET(sources track.cpp mht.cpp mhtStructs.cpp Clusters.cpp odometry.cpp total_velocity.cpp Groups.cpp
prediction_bhmip.cpp scene_abstract.cpp iri_geometry.cpp  )
# application header files
SET(headers ../include/track.h ../include/mht.h ../include/mhtStructs.h ../include/Clusters.h ../include/odometry.h ../include/total_velocity.h ../include/Groups.h ../include/prediction_bhmip.h ../include/scene_abstract.h ../include/iri_geometry.h )

# locate the necessary dependencies
#find_package(people_prediction REQUIRED)
find_package(Eigen3 REQUIRED)

# add the necessary include directories
INCLUDE_DIRECTORIES(../include)
#INCLUDE_DIRECTORIES(${people_prediction_INCLUDE_DIRS})
INCLUDE_DIRECTORIES(${Eigen3_INCLUDE_DIRS})
# create the shared library
ADD_LIBRARY(people_tracking_mht SHARED ${sources})
ADD_LIBRARY(people_tracking_mht SHARED ${sources_scene_elements})

#TARGET_LINK_LIBRARIES(people_tracking_mht ${people_prediction_LIBRAIES})

INSTALL(FILES ${headers_scene_elements} DESTINATION include/iri/${PROJECT_NAME}/scene_elements)

# link necessary libraries
INSTALL(TARGETS people_tracking_mht
        RUNTIME DESTINATION bin
        LIBRARY DESTINATION lib/iri/${PROJECT_NAME}
        ARCHIVE DESTINATION lib/iri/${PROJECT_NAME})
INSTALL(FILES ${headers} DESTINATION include/iri/${PROJECT_NAME})
INSTALL(FILES ../Findpeople_tracking_mht.cmake DESTINATION ${CMAKE_ROOT}/Modules/)
ADD_SUBDIRECTORY(examples)
