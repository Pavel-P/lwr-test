cmake_minimum_required(VERSION 2.8.3)
project(lwr_descartes_demo)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(gazebo REQUIRED)

## System dependencies are found with CMake's conventions
find_package(catkin REQUIRED COMPONENTS
    descartes_core
    descartes_moveit
    descartes_trajectory
    descartes_planner
    trajectory_msgs
    gazebo_ros
)

catkin_package(
  INCLUDE_DIRS #include
  LIBRARIES #lwr_descartes_demo
  CATKIN_DEPENDS #other_catkin_pkg
  DEPENDS gazebo_ros #system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
    include
    /usr/local/ulapi/include
    ${catkin_INCLUDE_DIRS}
    ${GAZEBO_INCLUDE_DIRS}
)

#demo
add_executable(${PROJECT_NAME} src/demo.cpp)
target_link_libraries(${PROJECT_NAME}
   ${catkin_LIBRARIES}
)
set_target_properties(${PROJECT_NAME}
   PROPERTIES OUTPUT_NAME demo
   PREFIX "")

#sm_server_demo
add_executable(sm_server_demo src/sm_server_demo.cpp)
target_link_libraries(sm_server_demo
    ${catkin_LIBRARIES}
    /usr/local/ulapi/lib/libulapi.a
    )
set_target_properties(${PROJECT_NAME}
   PROPERTIES OUTPUT_NAME sm_server_demo
   PREFIX "")

#sm_client_demo
add_executable(sm_client_demo src/sm_client_demo.cpp)
target_link_libraries(sm_client_demo
    ${catkin_LIBRARIES}
    /usr/local/ulapi/lib/libulapi.a
    )
set_target_properties(${PROJECT_NAME}
   PROPERTIES OUTPUT_NAME sm_client_demo
   PREFIX "")


## Mark executables and/or libraries for installation
 install(TARGETS ${PROJECT_NAME} sm_server_demo sm_client_demo
   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Mark cpp header files for installation
 install(DIRECTORY include/${PROJECT_NAME}/
   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
   FILES_MATCHING PATTERN "*.h"
   PATTERN ".svn" EXCLUDE
)
