cmake_minimum_required(VERSION 3.0.2)
project(robotics_project_one)


find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  message_generation
)

add_message_files(
  FILES
  WheelSpeeds.msg
)

add_service_files(
  FILES
  Reset_odometry.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
)


catkin_package(CATKIN_DEPENDS message_runtime)

include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(odom src/odom.cpp)
add_dependencies(odom ${catkin_EXPORTED_TARGETS})
target_link_libraries(odom ${catkin_LIBRARIES})

add_executable(compute_control src/compute_control.cpp)
add_dependencies(compute_control ${catkin_EXPORTED_TARGETS})
target_link_libraries(compute_control ${catkin_LIBRARIES})


