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

add_executable(base_link_navigation_client src/base_link_navigation_client.cpp)
target_link_libraries(base_link_navigation_client ${catkin_LIBRARIES})
add_dependencies(base_link_navigation_client ${catkin_EXPORTED_TARGETS})

add_executable(map_navigation_client src/map_navigation_client.cpp)
target_link_libraries(map_navigation_client ${catkin_LIBRARIES})
add_dependencies(map_navigation_client ${catkin_EXPORTED_TARGETS})

add_executable(laser_publisher src/laser_publisher.cpp)
target_link_libraries(laser_publisher ${catkin_LIBRARIES})
add_dependencies(laser_publisher ${catkin_EXPORTED_TARGETS})

add_executable(laser_tf_broadcaster src/laser_tf_broadcaster.cpp)
target_link_libraries(laser_tf_broadcaster ${catkin_LIBRARIES})
add_dependencies(laser_tf_broadcaster ${catkin_EXPORTED_TARGETS})

add_executable(pid_controller src/pid_controller.cpp)
target_link_libraries(pid_controller ${catkin_LIBRARIES})
add_dependencies(pid_controller ${catkin_EXPORTED_TARGETS})

add_executable(odometry_publisher src/odometry_publisher.cpp)
target_link_libraries(odometry_publisher ${catkin_LIBRARIES})
add_dependencies(odometry_publisher ${catkin_EXPORTED_TARGETS})

add_executable(move_base_recover src/move_base_recover.cpp)
target_link_libraries(move_base_recover ${catkin_LIBRARIES})
add_dependencies(move_base_recover ${catkin_EXPORTED_TARGETS})

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)
