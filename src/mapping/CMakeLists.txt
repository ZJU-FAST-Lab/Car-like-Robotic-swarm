cmake_minimum_required(VERSION 3.10.0)
project(mapping)

find_package(catkin REQUIRED COMPONENTS
  # message_generation
  std_msgs
  roscpp
  rospy
  sensor_msgs
  nav_msgs
  message_filters
  pcl_conversions
  # airsim_ros_pkgs
)
message("==============================================")
message(${catkin_INCLUDE_DIRS})
# add_message_files(
#   FILES
#   GimbalAngleEulerCmd.msg
#   GimbalAngleQuatCmd.msg
# )

# add_service_files(
#   FILES
#   SetGPSPosition.srv
#   Takeoff.srv
#   TakeoffGroup.srv
#   Land.srv
#   LandGroup.srv
#   Reset.srv
#   SetLocalPosition.srv
# )

# generate_messages(
#   DEPENDENCIES
#   std_msgs
#   sensor_msgs
#   nav_msgs
# )

find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES mapping
  # CATKIN_DEPENDS message_generation   
  CATKIN_DEPENDS std_msgs roscpp rospy sensor_msgs nav_msgs
)

include_directories(
  include
  ${PROJECT_SOURCE_DIR}/include
  ${catkin_INCLUDE_DIRS}
  ${Eigen3_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

link_directories(${PCL_LIBRARY_DIRS})

add_library( mapping 
    src/mapping.cpp 
    src/raycast.cpp
)

target_link_libraries(mapping ${catkin_LIBRARIES} ${PCL_LIBRARIES})


# add_dependencies(mapping_node  ${${PROJECT_NAME}_EXPORTED_TARGETS})

# add_dependencies(mapping mapping_generate_messages_cpp)


# install(TARGETS
#     #list of nodes
#     airsim_node
#     pd_position_controller_simple_node
#     DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

# install(TARGETS 
#     #list of shared libraries
#     airsim_ros
#     ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#     LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
# )

# install(FILES 
#   #list of necessary files (xml...)
#   README.md
#   # settings.json
#   # nodelet_plugins.xml
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

# # for shipping binaries. run `$ catin_make install`
# install(DIRECTORY
#   launch
#   rviz
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
#   # PATTERN ".git" EXCLUDE
# )
