cmake_minimum_required(VERSION 3.0.2)
project(robotics_project_one)


find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  message_generation
  tf2
  tf2_ros
  dynamic_reconfigure
)

add_message_files(
  FILES
  WheelSpeeds.msg
)

add_service_files(
  FILES
  Reset_odometry.srv
  Set_compute_control_param.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

generate_dynamic_reconfigure_options(
  cfg/odometryIntegration.cfg
  
)


catkin_package(CATKIN_DEPENDS message_runtime)

include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(compute_odometry src/compute_odometry.cpp)
add_dependencies(compute_odometry ${catkin_EXPORTED_TARGETS})
target_link_libraries(compute_odometry ${catkin_LIBRARIES})

add_executable(compute_control src/compute_control.cpp)
add_dependencies(compute_control ${catkin_EXPORTED_TARGETS})
target_link_libraries(compute_control ${catkin_LIBRARIES})

add_executable(tf2_broad src/broadcaster_tf2.cpp)
add_dependencies(tf2_broad ${catkin_EXPORTED_TARGETS})
target_link_libraries(tf2_broad ${catkin_LIBRARIES})

add_executable(static_broadcaster src/static_tf_broadcaster.cpp)
add_dependencies(static_broadcaster ${catkin_EXPORTED_TARGETS})
target_link_libraries(static_broadcaster ${catkin_LIBRARIES})

