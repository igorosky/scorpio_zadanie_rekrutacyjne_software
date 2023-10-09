cmake_minimum_required(VERSION 3.0.2)
project(virtual_dc_motor)

find_package(catkin REQUIRED COMPONENTS
  rospy
  roscpp
  std_msgs
)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
  include
  include/virtual_dc_motor
  include/speed_controller
)

catkin_install_python(PROGRAMS
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

add_executable(virtual_dc_motor src/virtual_dc_motor.cpp)

target_link_libraries(virtual_dc_motor ${catkin_LIBRARIES})

add_executable(rpm src/rpm.cpp)

target_link_libraries(rpm ${catkin_LIBRARIES})

# add_executable(speed_data_collector src/speed_data_collector.cpp)

# target_link_libraries(speed_data_collector ${catkin_LIBRARIES})

add_executable(speed_controller src/pid.cpp src/speed_controller.cpp)

target_link_libraries(speed_controller ${catkin_LIBRARIES})
