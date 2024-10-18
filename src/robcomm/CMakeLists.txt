cmake_minimum_required(VERSION 3.8)
project(robcomm VERSION 0.1.0 LANGUAGES CXX)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)

# specify include directories
include_directories(include)

# Get source files and header files
file(GLOB ROBCOMM_SOURCES src/*.cpp)
file(GLOB ROBCOMM_HEADERS include/robcomm/*.hpp)

# Add shared library
add_library(robcomm SHARED ${ROBCOMM_SOURCES})

# Set header file directory
target_include_directories(robcomm PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_LIST_DIR}/include>
  $<INSTALL_INTERFACE:include>)

# Link dependency packages
ament_target_dependencies(robcomm rclcpp geometry_msgs tf2_ros)

# Install targets and configuration
install(TARGETS robcomm EXPORT robcommConfig DESTINATION lib)
install(DIRECTORY include/ DESTINATION include)

# Package configuration
ament_export_include_directories(include)
ament_export_libraries(robcomm)
ament_export_dependencies(rclcpp geometry_msgs tf2_ros)

ament_package()
