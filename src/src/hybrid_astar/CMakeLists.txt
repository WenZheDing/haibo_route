cmake_minimum_required(VERSION 3.0.2)
project(hybrid_astar)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  roslib
  std_msgs
  tf
  nav_msgs
  geometry_msgs
)

set(SOURCES
    ${CMAKE_CURRENT_SOURCE_DIR}/src/algorithm.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/node2d.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/node3d.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/collisiondetection.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/planner.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/path.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/smoother.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/visualize.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/dynamicvoronoi.cpp #Boris Lau, Christoph Sprunk, Wolfram Burgard
    ${CMAKE_CURRENT_SOURCE_DIR}/src/bucketedqueue.cpp #Boris Lau, Christoph Sprunk, Wolfram Burgard
    ${CMAKE_CURRENT_SOURCE_DIR}/src/reedsshepp.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/dubins.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/constants.cpp
    )
set(HEADERS
    ${CMAKE_CURRENT_SOURCE_DIR}/include/hybrid_astar/algorithm.h
    ${CMAKE_CURRENT_SOURCE_DIR}/include/hybrid_astar/node2d.h
    ${CMAKE_CURRENT_SOURCE_DIR}/include/hybrid_astar/node3d.h
    ${CMAKE_CURRENT_SOURCE_DIR}/include/hybrid_astar/collisiondetection.h
    ${CMAKE_CURRENT_SOURCE_DIR}/include/hybrid_astar/planner.h
    ${CMAKE_CURRENT_SOURCE_DIR}/include/hybrid_astar/path.h
    ${CMAKE_CURRENT_SOURCE_DIR}/include/hybrid_astar/smoother.h
    ${CMAKE_CURRENT_SOURCE_DIR}/include/hybrid_astar/vector2d.h
    ${CMAKE_CURRENT_SOURCE_DIR}/include/hybrid_astar/visualize.h
    ${CMAKE_CURRENT_SOURCE_DIR}/include/hybrid_astar/helper.h
    ${CMAKE_CURRENT_SOURCE_DIR}/include/hybrid_astar/constants.h
    ${CMAKE_CURRENT_SOURCE_DIR}/include/hybrid_astar/lookup.h
    ${CMAKE_CURRENT_SOURCE_DIR}/include/hybrid_astar/gradient.h #Andrew Noske
    #${CMAKE_CURRENT_SOURCE_DIR}/include/dubins.h #Andrew Walker
    ${CMAKE_CURRENT_SOURCE_DIR}/include/hybrid_astar/dynamicvoronoi.h #Boris Lau, Christoph Sprunk, Wolfram Burgard
    ${CMAKE_CURRENT_SOURCE_DIR}/include/hybrid_astar/bucketedqueue.h #Boris Lau, Christoph Sprunk, Wolfram Burgard
    ${CMAKE_CURRENT_SOURCE_DIR}/include/hybrid_astar/point.h #Boris Lau, Christoph Sprunk, Wolfram Burgard
    ${CMAKE_CURRENT_SOURCE_DIR}/include/hybrid_astar/reedsshepp.h
    ${CMAKE_CURRENT_SOURCE_DIR}/include/hybrid_astar/dubins.h
    )

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)
find_package(ompl REQUIRED)
if(NOT OMPL_FOUND)
    message(AUTHOR_WARNING,"Open Motion Planning Library not found")
endif(NOT OMPL_FOUND)



###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories( include 
    ${catkin_INCLUDE_DIRS}
    ${OMPL_INCLUDE_DIRS}
    include/hybrid_astar
)

catkin_package(
 INCLUDE_DIRS include
 LIBRARIES hybrid_astar_lib
#  CATKIN_DEPENDS roscpp
#  DEPENDS system_lib
)
## Declare a C++ library
add_library(hybrid_astar_lib
  ${SOURCES}
  ${HEADERS}
)

target_link_libraries(hybrid_astar_lib
  ${catkin_LIBRARIES}
  ${OMPL_LIBRARIES}
  jsoncpp
)

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
add_dependencies(
    # ${PROJECT_NAME} 
    # ${${PROJECT_NAME}_EXPORTED_TARGETS} 
    ${catkin_EXPORTED_TARGETS})

## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide
add_executable(tf_broadcaster src/tf_broadcaster.cpp)
target_link_libraries(tf_broadcaster ${catkin_LIBRARIES})

add_executable(hybrid_astar_node src/main.cpp ${HEADERS} ${SOURCES})
target_link_libraries(hybrid_astar_node hybrid_astar_lib)

install(TARGETS hybrid_astar_lib
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)

## Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
)

