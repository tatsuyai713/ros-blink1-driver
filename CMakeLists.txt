cmake_minimum_required(VERSION 3.5)
project(ros_blink1_driver)

set(CMAKE_C_STANDARD 99)

# Set the default USBLIB_TYPE if not provided
if(NOT DEFINED USBLIB_TYPE)
    set(USBLIB_TYPE "HIDAPI")
endif()

# Set the default HIDAPI_TYPE if not provided
if(NOT DEFINED HIDAPI_TYPE)
    set(HIDAPI_TYPE "HIDRAW")
endif()

# Define the source files
set(SRC_FILES
    src/blink1-lib.c
    src/hiddata.c
)

# Add source files
if(HIDAPI_TYPE STREQUAL "HIDRAW")
    list(APPEND SRC_FILES src/hidapi/linux/hid.c)
elseif(HIDAPI_TYPE STREQUAL "LIBUSB")
    list(APPEND SRC_FILES src/hidapi/libusb/hid.c)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

add_library(blink1-lib ${SRC_FILES})

if(USBLIB_TYPE STREQUAL "HIDDATA")
    target_compile_definitions(blink1-lib PRIVATE USE_HIDDATA)
elseif(USBLIB_TYPE STREQUAL "HIDAPI")
    target_compile_definitions(blink1-lib PRIVATE USE_HIDAPI)
endif()

find_package(PkgConfig REQUIRED)
pkg_check_modules(LIBUSB libusb-1.0 REQUIRED)
pkg_check_modules(UDEV REQUIRED libudev)

# Include directories for blink1-lib
target_include_directories(blink1-lib PRIVATE ${LIBUSB_INCLUDE_DIRS} include src/hidapi/hidapi)

# Add executable target
add_executable(ros_blink1_driver_node src/ros_blink1_driver.cpp)

# Link blink1-lib to ros_blink1_driver_node
target_link_libraries(ros_blink1_driver_node blink1-lib ${UDEV_LIBRARIES} ${LIBUSB_LIBRARIES})

# Include directories for ros_blink1_driver_node
target_include_directories(ros_blink1_driver_node PRIVATE include ${CMAKE_CURRENT_SOURCE_DIR} ${LIBUSB_INCLUDE_DIRS})

ament_target_dependencies(ros_blink1_driver_node rclcpp std_msgs)

# Install targets
install(TARGETS
  ros_blink1_driver_node
  DESTINATION lib/${PROJECT_NAME})

# Install launch files
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

# Install scripts
install(DIRECTORY
  scripts
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
