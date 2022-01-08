cmake_minimum_required(VERSION 3.0.2)
project(OccupiedGridObstacleAvoidanceRos)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
)

catkin_package(
)

###########
## Build ##
###########

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

find_package(ompl REQUIRED)
include_directories(${OMPL_INCLUDE_DIRS})
link_directories(${OMPL_LIBRARY_DIRS})
add_definitions(${OMPL_DEFINITIONS})

FIND_PACKAGE(Boost COMPONENTS system REQUIRED)
include_directories(${Boost_INCLUDE_DIR})

# For Eigen
find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})

add_executable(OccupiedGridObstacleAvoidanceRos src/main.cpp src/MotionPlanner.cpp src/OptimizationObjectives.cpp)

target_link_libraries(OccupiedGridObstacleAvoidanceRos
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
  ${OMPL_LIBRARIES}
  ${Boost_PROGRAM_OPTIONS_LIBRARY}
)