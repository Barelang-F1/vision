cmake_minimum_required(VERSION 3.10)
project(detection)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  sensor_msgs
)
find_package(CUDA)

# Check for CUDA
if (NOT CUDA_FOUND)
    message(FATAL_ERROR "CUDA not found!")
endif()

# Include directories
include_directories(
  ${catkin_INCLUDE_DIRS}
  /usr/local/include/jetson-inference
  /usr/local/include/jetson-utils
  ${CUDA_INCLUDE_DIRS}
)

# Set C++ standard
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Declare a catkin package
catkin_package(
  CATKIN_DEPENDS roscpp std_msgs sensor_msgs
)

add_executable(korban src/korban.cpp)

# Specify the libraries to link against
target_link_libraries(korban
  ${catkin_LIBRARIES}
  jetson-inference
  jetson-utils
  ${CUDA_LIBRARIES}
)
