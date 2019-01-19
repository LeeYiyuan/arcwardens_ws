cmake_minimum_required(VERSION 2.8.12)
add_compile_options(-std=c++11)
project(car)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  std_msgs
  serial
  tf2
  tf2_ros
  tf2_msgs
  tf2_sensor_msgs
  tf2_geometry_msgs
)
catkin_package(INCLUDE_DIRS CATKIN_DEPENDS
  roscpp
  sensor_msgs
  std_msgs
  serial
  tf2
  tf2_ros
  tf2_msgs
  tf2_sensor_msgs
  tf2_geometry_msgs
)

include_directories(${catkin_INCLUDE_DIRS})

add_executable(joystick_teleop src/joystick_teleop.cpp)
target_link_libraries(joystick_teleop ${catkin_LIBRARIES})
