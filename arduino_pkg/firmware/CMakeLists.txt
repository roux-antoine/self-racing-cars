cmake_minimum_required(VERSION 3.0.2)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

include_directories(${ROS_LIB_DIR})

# Remove this if using an Arduino without native USB (eg, other than Leonardo)
add_definitions(-DUSB_CON)

generate_arduino_firmware(aim
  SRCS aim.cpp ${ROS_LIB_DIR}/time.cpp
  BOARD mega
  PORT /dev/ttyACM0
)
