cmake_minimum_required(VERSION 3.0.2)
project(robotics_project_one)


find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
)


catkin_package()

include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(odom src/odom.cpp)
target_link_libraries(odom ${catkin_LIBRARIES})


