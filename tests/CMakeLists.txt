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

# Only build tests if testing is enabled
if(CATKIN_ENABLE_TESTING)

# Find required packages for testing
find_package(catkin REQUIRED COMPONENTS
  rostest
  roscpp
  sensor_msgs
  cv_bridge
  image_transport
  movidius_ncs_lib
)

find_package(OpenCV REQUIRED)
find_package(GTest REQUIRED)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${GTEST_INCLUDE_DIRS}
  ../Lib/include
)

# Unit tests
set(UNIT_TESTS
  test_device
  test_tensor
  test_result
  test_ncs_manager
  test_exception_handling
  test_performance
)

# Integration tests
set(INTEGRATION_TESTS
  test_image_classification
  test_image_detection
  test_stream_processing
  test_multi_device
)

# Build unit tests
foreach(test ${UNIT_TESTS})
  add_executable(${test} ${test}.cpp)
  target_link_libraries(${test}
    ${catkin_LIBRARIES}
    ${OpenCV_LIBRARIES}
    ${GTEST_LIBRARIES}
    movidius_ncs_lib
    pthread
  )
  add_test(${test} ${test})
endforeach()

# Build integration tests
foreach(test ${INTEGRATION_TESTS})
  add_executable(${test} ${test}.cpp)
  target_link_libraries(${test}
    ${catkin_LIBRARIES}
    ${OpenCV_LIBRARIES}
    ${GTEST_LIBRARIES}
    movidius_ncs_lib
    pthread
  )
endforeach()

# Add rostests
add_rostest(test_image_classification.test)
add_rostest(test_image_detection.test)
add_rostest(test_stream_processing.test)

# Install test files
install(FILES
  test_image_classification.test
  test_image_detection.test
  test_stream_processing.test
  test_all.test
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/tests
)

endif(CATKIN_ENABLE_TESTING)
