cmake_minimum_required(VERSION 2.8.3)

include_directories(${ROS_LIB_DIR})

###########
## Build ##
###########

add_definitions(-DUSB_CON)

generate_arduino_firmware(voltage_monitor
  BOARD nano328
  SRCS voltage_monitor.cpp ${ROS_LIB_DIR}/time.cpp
  HDRS calib.h
  PORT /dev/arduino_voltage
  PROGRAMMER avrispmkii
)
