# Copyright (c) 2017-2025 Intel Corporation
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#      http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

cmake_minimum_required(VERSION 3.0.2)
project(movidius_ncs_project)

# Set C++ standard
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# Set build type if not specified
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

# Find required packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  std_msgs
  cv_bridge
  image_transport
  camera_info_manager
  roslint
  rostest
)

# Find OpenCV (required for image processing)
find_package(OpenCV REQUIRED)

# Find Boost (required for string operations)
find_package(Boost REQUIRED COMPONENTS system filesystem)

# Catkin package configuration
catkin_package(
  INCLUDE_DIRS
    include
    ${OpenCV_INCLUDE_DIRS}
  LIBRARIES
    movidius_ncs_lib
  CATKIN_DEPENDS
    roscpp
    sensor_msgs
    std_msgs
    cv_bridge
    image_transport
  DEPENDS
    OpenCV
    Boost
)

# Include directories
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
)

# Subdirectories
add_subdirectory(Lib)
add_subdirectory(Imaging)

# Example applications
add_subdirectory(examples)

# Tests
if(CATKIN_ENABLE_TESTING)
  add_subdirectory(tests)
endif()

# Install configuration files
install(DIRECTORY config/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config
  FILES_MATCHING PATTERN "*.yaml"
)

install(DIRECTORY Launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
  FILES_MATCHING PATTERN "*.yaml"
)

install(DIRECTORY data/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/data
  FILES_MATCHING PATTERN "*"
)

# Install documentation
install(DIRECTORY doc/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/doc
  FILES_MATCHING PATTERN "*.md"
)

# Setup ROS environment
catkin_python_setup()

# Print configuration summary
message(STATUS "=========================================")
message(STATUS "Movidius NCS Project Configuration:")
message(STATUS "  Build type: ${CMAKE_BUILD_TYPE}")
message(STATUS "  C++ Standard: ${CMAKE_CXX_STANDARD}")
message(STATUS "  OpenCV version: ${OpenCV_VERSION}")
message(STATUS "  Boost version: ${Boost_VERSION}")
message(STATUS "  Install prefix: ${CMAKE_INSTALL_PREFIX}")
message(STATUS "  ROS workspace: $ENV{ROS_WORKSPACE}")
message(STATUS "=========================================")
