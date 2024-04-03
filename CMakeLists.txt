cmake_minimum_required(VERSION 3.0.2)
project(ackermann_unicycle)


find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  nav_msgs
  geometry_msgs
)

## System dependencies are found with CMake's conventions

## Build the node
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)



################################################
## Declare ROS messages, services and actions ##
################################################

generate_messages(DEPENDENCIES std_msgs nav_msgs geometry_msgs)

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

###################################
## catkin specific configuration ##
###################################

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ackermann_unicycle
  CATKIN_DEPENDS roscpp nav_msgs geometry_msgs 

)


###########
## Build ##
###########

include_directories(

  ${catkin_INCLUDE_DIRS}
)


 add_executable(unicycle src/au_node.cpp src/ackermann_unicycle.cpp)
 target_link_libraries(unicycle ${catkin_LIBRARIES}
                                ${EIGEN3_LIBRARIES})


add_dependencies(unicycle ${catkin_EXPORTED_TARGETS} ${EIGEN3_EXPORTED_TARGETS})

