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
  darknet_ros
)
add_message_files (
   FILES
   person.msg
)

generate_messages(
   DEPENDENCIES
   std_msgs  # Or other packages containing msgs
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
  darknet_ros
)


include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(joystick_teleop src/joystick_teleop.cpp)
target_link_libraries(joystick_teleop ${catkin_LIBRARIES})

add_executable(low_level_controller src/low_level_controller.cpp)
target_link_libraries(low_level_controller ${catkin_LIBRARIES})

add_executable(arcwarden_controller src/arcwarden_controller.cpp)
target_link_libraries(arcwarden_controller ${catkin_LIBRARIES})

add_executable(person_depth src/person_depth.cpp)
target_link_libraries(person_depth ${catkin_LIBRARIES})
