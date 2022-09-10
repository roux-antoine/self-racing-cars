# Setup

The package has been setup using the instructions in http://wiki.ros.org/rosserial_arduino/Tutorials/CMake

Some notes:
* since we use Kinetic, add `add_compile_options(-std=c++11)` in all CMakeLists.txt

# How to build and upload

## How to build

In `firmware/CMakeLists.txt`:
* Make sure that the board is correct (uno vs mega)
* Make sure that the port where the board is connected is correct

Then run: `catkin build arduino_pkg`

## How to upload

`catkin build --no-deps  arduino_pkg --make-args arduino_pkg_firmware_aim-upload`

