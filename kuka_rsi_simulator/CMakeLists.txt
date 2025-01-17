cmake_minimum_required(VERSION 3.1.3)
project(kuka_rsi_simulator)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS roscpp std_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

catkin_install_python(PROGRAMS scripts/kuka_rsi_simulator
	DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

add_executable(kuka_rsi_simulator_cpp src/kuka_rsi_simulator.cpp)
target_link_libraries(kuka_rsi_simulator_cpp ${catkin_LIBRARIES} ${TinyXML_LIBRARIES})