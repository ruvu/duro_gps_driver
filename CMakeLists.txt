cmake_minimum_required(VERSION 2.8.3)
project(duro_ros)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  nav_msgs
  roscpp
  std_msgs
  tf2
  tf2_msgs
  tf2_ros
  tf2_geometry_msgs
)

find_library(SBP sbp)

###################################
## catkin specific configuration ##
###################################
catkin_package(
  INCLUDE_DIRS include
)

###########
## Build ##
###########

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

## Declare a C++ library
add_library(duro_ros
  src/duro_ros.cpp
)

target_link_libraries(duro_ros
  ${SBP}
)

add_executable(duro_node src/duro_node.cpp)

target_link_libraries(duro_node duro_ros ${catkin_LIBRARIES})
