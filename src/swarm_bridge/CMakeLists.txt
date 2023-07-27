cmake_minimum_required(VERSION 2.8.3)
project(swarm_bridge)

set(CMAKE_BUILD_TYPE "Release")
ADD_COMPILE_OPTIONS(-std=c++14 )
# set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")

find_package(Eigen3 REQUIRED)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  nav_msgs
  geometry_msgs
  message_generation
  # traj_planner
)


add_message_files(
  FILES
  SingleMinco.msg
  MincoTraj.msg
  Trajectory.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
  nav_msgs
)

catkin_package(
 CATKIN_DEPENDS roscpp std_msgs nav_msgs geometry_msgs
#  DEPENDS system_lib
)

add_library(swarm_bridge
  src/bridge_node.cpp
)

add_dependencies(swarm_bridge 
  
  ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS}

  )

include_directories(
  SYSTEM
  ${catkin_INCLUDE_DIRS} 
  ${PLAN_UTILS_INCLUDE_DIRS}
  ${PROJECT_SOURCE_DIR}/include
  ${EIGEN3_INCLUDE_DIR}
)

add_executable(bridge_node
  src/bridge_node.cpp 
  )
target_link_libraries(bridge_node
  swarm_bridge
  ${catkin_LIBRARIES}
  )
