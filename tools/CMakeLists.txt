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

# Find required packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  cv_bridge
  movidius_ncs_lib
)

find_package(OpenCV REQUIRED)

# Include directories
include_directories(
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ../Lib/include
)

# Tool applications
set(TOOL_APPS
  performance_analyzer
  model_converter
  device_manager
  thermal_monitor
)

# Build each tool
foreach(app ${TOOL_APPS})
  add_executable(${app} ${app}.cpp)
  target_link_libraries(${app}
    ${catkin_LIBRARIES}
    ${OpenCV_LIBRARIES}
    movidius_ncs_lib
  )
  set_target_properties(${app} PROPERTIES
    CXX_STANDARD 14
    CXX_STANDARD_REQUIRED ON
  )
endforeach()

# Install tools
install(TARGETS ${TOOL_APPS}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
