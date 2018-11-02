cmake_minimum_required(VERSION 2.8.3)
project(watchdog)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  rosserial_arduino
  rosserial_client
  std_msgs
)

###########################################
## Rebuild Calib Header from Config File ##
###########################################

add_custom_target(
  calib_header ALL
  COMMAND ${CMAKE_CURRENT_SOURCE_DIR}/scripts/firmware/build_calib_header.sh
  WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/scripts/firmware/
)

###################
## catkin config ##
###################

catkin_package(
  CATKIN_DEPENDS std_msgs
  DEPENDS calib_header
)

###########
## Build ##
###########

rosserial_generate_ros_lib(
  PACKAGE rosserial_arduino
  SCRIPT make_libraries.py
)

rosserial_configure_client(
  DIRECTORY firmware
  TOOLCHAIN_FILE ${ROSSERIAL_ARDUINO_TOOLCHAIN}
)

rosserial_add_client_target(firmware voltage_monitor ALL)
rosserial_add_client_target(firmware voltage_monitor-upload)

#############
## Install ##
#############

# Rosserial will install things automagically
# The watchdog script should not be installed as an executable

