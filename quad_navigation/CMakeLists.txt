cmake_minimum_required(VERSION 2.8.3)
project(quad_navigation)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  nav_msgs
  roscpp
  rospy
  sensor_msgs
  std_msgs
  tf
  mavros_msgs
)

find_package(cmake_modules REQUIRED)

catkin_package(
  LIBRARIES quad_navigation
  CATKIN_DEPENDS nav_msgs roscpp rospy sensor_msgs std_msgs tf
)


include_directories(
  ${catkin_INCLUDE_DIRS}
)


add_executable(offb_node src/offb_node.cpp)
target_link_libraries(offb_node ${catkin_LIBRARIES})

add_executable(offb_velocity_node src/offb_velocity_node.cpp)
target_link_libraries(offb_velocity_node ${catkin_LIBRARIES})

add_executable(mocap_node src/mocap_node.cpp)
target_link_libraries(mocap_node ${catkin_LIBRARIES})
