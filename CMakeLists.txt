cmake_minimum_required(VERSION 3.0.2)

project(mpu6050_dmp_ros)

find_package(catkin REQUIRED COMPONENTS
  rospy
  sensor_msgs
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS rospy sensor_msgs std_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

catkin_install_python(PROGRAMS
  scripts/imu_publisher
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
