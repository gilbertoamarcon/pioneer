cmake_minimum_required(VERSION 2.8.3)
project(pioneer)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  std_msgs
  geometry_msgs
  sensor_msgs
  nav_msgs
  move_base_msgs
  actionlib
  roscpp
  rospy
  tf
)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(teleop_joy src/teleop_joy.cpp)
target_link_libraries(teleop_joy ${catkin_LIBRARIES})
add_dependencies(teleop_joy ${catkin_EXPORTED_TARGETS})

add_executable(laser_publisher src/laser_publisher.cpp)
target_link_libraries(laser_publisher ${catkin_LIBRARIES})
add_dependencies(laser_publisher ${catkin_EXPORTED_TARGETS})

add_executable(laser_tf_broadcaster src/laser_tf_broadcaster.cpp)
target_link_libraries(laser_tf_broadcaster ${catkin_LIBRARIES})
add_dependencies(laser_tf_broadcaster ${catkin_EXPORTED_TARGETS})

add_executable(odometry_publisher src/odometry_publisher.cpp)
target_link_libraries(odometry_publisher ${catkin_LIBRARIES})
add_dependencies(odometry_publisher ${catkin_EXPORTED_TARGETS})

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)
