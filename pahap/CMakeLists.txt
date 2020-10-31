cmake_minimum_required(VERSION 2.8.3)
project(pahap)


add_compile_options(-std=c++11)
find_package(PCL 1.8 REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

# find system dependencies
find_package(Eigen3 REQUIRED)
include_directories(${Eigen3_INCLUDE_DIRS})
link_directories(${Eigen3_LIBRARY_DIRS})
add_definitions(${Eigen3_DEFINITIONS})

find_package(Boost REQUIRED)

# find_package(OpenCV REQUIRED)

# find catkin packages
find_package(catkin REQUIRED COMPONENTS
  message_generation
  std_msgs
  roscpp
  sensor_msgs
  pcl_conversions
  rospy
  cmake_modules
  geometry_msgs
  eigen_conversions
  tf
  urdf
  moveit_msgs
  trajectory_msgs
  moveit_core
  moveit_ros_planning
  moveit_ros_planning_interface
  moveit_visual_tools
  rviz_visual_tools
  # cv_bridge
  # image_transport
  # OpenCV
)

add_message_files(
  FILES
  controllerCheck.msg cluster.msg
)

# add_definitions("-DUSE_ROS")

## Generate services in the 'srv' folder -> this one must be above generate_messages and catkin_package
add_service_files(
  FILES magarr_controller.srv image_cmd.srv pointcloud_cmd.srv pointcloud_nav.srv pcl_segment.srv pcl_boundary.srv displayData.srv graph_estimate.srv rrt.srv
  )

## Generate added messages and services with any dependencies listed here
generate_messages(DEPENDENCIES geometry_msgs sensor_msgs std_msgs)
# generate_messages(DEPENDENCIES geometry_msgs sensor_msgs std_msgs)


catkin_package(
  INCLUDE_DIRS include
  LIBRARIES pahap graphEstimate
  CATKIN_DEPENDS cmake_modules eigen_conversions geometry_msgs message_runtime roscpp sensor_msgs std_msgs 
  moveit_msgs trajectory_msgs moveit_core moveit_ros_planning  moveit_ros_planning_interface 
  DEPENDS Eigen PCL Boost
)
# do not wildcard install files since the root folder of the package will contain a debian folder for releasing

#####################################################################################
# Build
#####################################################################################
## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include 
  ${catkin_INCLUDE_DIRS} 
  ${PCL_INCLUDE_DIRS} 
  ${Boost_INCLUDE_DIRS}
)

add_executable(pahap src/pahap.cpp)
# This line tells catkin to first build the messages in turtlesim before trying to build this node.
# That is because we rely on the turtlesim/Pose message, which becomes a header file after it's compiled
add_dependencies(pahap ${${PPROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS} pahap_gencpp)

# This links up the node with the compiled libraries catkin knows about.
# Basically, it allows the node to leverage ROS libraries.
target_link_libraries(pahap ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${Boost_LIBRARIES})

add_executable(pcl_pahap_client src/pcl_pahap_client.cpp)
add_dependencies(pcl_pahap_client ${${PPROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS} pahap_gencpp)
target_link_libraries(pcl_pahap_client ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${Boost_LIBRARIES})

add_executable(pahap_client_navi src/pahap_client_navi.cpp)
add_dependencies(pahap_client_navi ${${PPROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS} pahap_gencpp)
target_link_libraries(pahap_client_navi ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${Boost_LIBRARIES})

add_executable(graphEstimate src/graphEstimate.cpp)
add_dependencies(graphEstimate ${${PPROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS} pahap_gencpp graphEstimate_gencpp)
target_link_libraries(graphEstimate ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${Boost_LIBRARIES})


################################################################################
# Install
################################################################################
catkin_install_python(
  PROGRAMS scripts/segmentServer.py scripts/displayServer.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(TARGETS ${PROJECT_NAME}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(PROGRAMS
  scripts/segmentServer
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}  
)

# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
# )

# install(DIRECTORY launch
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

# install(FILES 99-open-manipulator-cdc.rules
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

################################################################################
# Test
################################################################################
