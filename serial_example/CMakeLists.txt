cmake_minimum_required(VERSION 2.8.3)
project(serial_example)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  serial
  std_msgs
  geometry_msgs
  nav_msgs
  tf
)

catkin_package(
  CATKIN_DEPENDS
    serial
    std_msgs
    
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

link_directories(~/catkin_ws/src/imu_tools/imu_filter_madgwick)



add_executable(serial_imu_node src/serial_imu_node.cpp)
target_link_libraries(serial_imu_node
  ${catkin_LIBRARIES})

add_executable(twist_to_motor src/twist_to_motor.cpp)
target_link_libraries(twist_to_motor ${catkin_LIBRARIES})

add_executable(serial_tx_motor_node src/serial_tx_motor_node.cpp)
target_link_libraries(serial_tx_motor_node ${catkin_LIBRARIES})

add_executable(serial_rx_motor_node src/serial_rx_motor_node.cpp)
target_link_libraries(serial_rx_motor_node ${catkin_LIBRARIES})

add_executable(odometry src/odometry.cpp)
target_link_libraries(odometry ${catkin_LIBRARIES})

add_executable(odometry_2 src/odometry_2.cpp)
target_link_libraries(odometry_2 ${catkin_LIBRARIES})
