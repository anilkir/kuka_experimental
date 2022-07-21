cmake_minimum_required(VERSION 3.1.3)
project(kuka_rsi_simulator)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs)

catkin_package()

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

catkin_install_python(PROGRAMS scripts/kuka_rsi_simulator
	DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})