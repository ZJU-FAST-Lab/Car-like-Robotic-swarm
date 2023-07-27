cmake_minimum_required(VERSION 2.8.3)
project(kinematics_simulator)

set(CMAKE_BUILD_TYPE "Release")
ADD_COMPILE_OPTIONS(-std=c++11 )
ADD_COMPILE_OPTIONS(-std=c++14 )
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  nav_msgs
  std_msgs
  visualization_msgs
  message_generation
)
find_package(Eigen3 REQUIRED)

set(PLAN_UTILS_INCLUDE_DIRS ${PROJECT_SOURCE_DIR}/include/plan_utils)
# set(GEO_UITLS_INCLUDE_DIRS ${PROJECT_SOURCE_DIR}/include/geo_utils2d)

add_message_files(FILES
  SingleMinco.msg
  MincoTraj.msg
  Trajectory.msg
)

generate_messages(DEPENDENCIES
	std_msgs
  geometry_msgs
  nav_msgs
)

catkin_package(
#  INCLUDE_DIRS include ${PLAN_UTILS_INCLUDE_DIRS} 
#  ${GEO_UTILS_INCLUDE_DIRs}
 CATKIN_DEPENDS roscpp std_msgs message_runtime
)

include_directories( include 
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
  ${PLAN_UTILS_INCLUDE_DIRS}
  # ${GEO_UTILS_INCLUDE_DIRs}
)

add_executable (kinematics_simulator_node src/kinematics_sim.cpp )
target_link_libraries(kinematics_simulator_node
  ${catkin_LIBRARIES})
