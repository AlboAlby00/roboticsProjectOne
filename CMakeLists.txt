cmake_minimum_required(VERSION 3.0.2)
project(roboticsProjectOne)


find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
)


catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES roboticsProjectOne
#  CATKIN_DEPENDS roscpp rospy std_msgs
#  DEPENDS system_lib
)

add_executable(odometryNode src/odom.cpp)

target_link_libraries(odometryNode ${catkin_LIBRARIES})

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)
