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
project(movidius_ncs_lib)

# Find required packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
)

# Find OpenCV (required for image processing)
find_package(OpenCV REQUIRED)

# Find Intel Movidius NCS SDK
find_path(MVNC_INCLUDE_DIR mvnc.h
  HINTS /opt/movidius/nc-sdk/include
  PATHS /usr/include /usr/local/include
)

find_library(MVNC_LIBRARY mvnc
  HINTS /opt/movidius/nc-sdk/lib
  PATHS /usr/lib /usr/local/lib
)

if(NOT MVNC_INCLUDE_DIR OR NOT MVNC_LIBRARY)
  message(WARNING "Intel Movidius NCS SDK not found. Please install it for full functionality.")
endif()

# Catkin package configuration
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS roscpp std_msgs
  DEPENDS OpenCV
)

# Include directories
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${MVNC_INCLUDE_DIR}
)

# Source files
set(SOURCES
  ../src cpp/device.cpp
  ../src cpp/exception.cpp
  ../src cpp/exception_util.cpp
  ../src cpp/graph.cpp
  ../src cpp/ncs.cpp
  ../src cpp/ncs_manager.cpp
  ../src cpp/result.cpp
  ../src cpp/tensor.cpp
)

# Header files
set(HEADERS
  include/lib/device.h
  include/lib/exception.h
  include/lib/exception_util.h
  include/lib/graph.h
  include/lib/mvnc_cpp.h
  include/lib/ncs.h
  include/lib/ncs_manager.h
  include/lib/result.h
  include/lib/tensor.h
)

# Create library
add_library(${PROJECT_NAME} ${SOURCES})

# Link libraries
target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
  ${MVNC_LIBRARY}
)

# Set compiler flags
set_target_properties(${PROJECT_NAME} PROPERTIES
  CXX_STANDARD 14
  CXX_STANDARD_REQUIRED ON
)

# Install headers
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

# Install library
install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# Export library for other packages to use
export(TARGETS ${PROJECT_NAME}
  FILE "${PROJECT_BINARY_DIR}/${PROJECT_NAME}Targets.cmake"
)

# Generate package configuration files
configure_file("${CMAKE_CURRENT_SOURCE_DIR}/cmake/${PROJECT_NAME}-config.cmake.in"
  "${PROJECT_BINARY_DIR}/${PROJECT_NAME}-config.cmake" @ONLY)

install(FILES
  "${PROJECT_BINARY_DIR}/${PROJECT_NAME}-config.cmake"
  DESTINATION "${CATKIN_PACKAGE_SHARE_DESTINATION}/cmake"
)
