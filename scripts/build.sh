#!/bin/bash

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

set -e

# Script configuration
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(dirname "$SCRIPT_DIR")"
BUILD_DIR="${PROJECT_ROOT}/build"
INSTALL_DIR="${PROJECT_ROOT}/install"

# Default values
BUILD_TYPE="Release"
JOBS=$(nproc 2>/dev/null || echo 4)
VERBOSE=0
CLEAN_BUILD=0
RUN_TESTS=0

# Parse command line arguments
while [[ $# -gt 0 ]]; do
  case $1 in
    -t|--build-type)
      BUILD_TYPE="$2"
      shift 2
      ;;
    -j|--jobs)
      JOBS="$2"
      shift 2
      ;;
    -v|--verbose)
      VERBOSE=1
      shift
      ;;
    -c|--clean)
      CLEAN_BUILD=1
      shift
      ;;
    --test)
      RUN_TESTS=1
      shift
      ;;
    -h|--help)
      echo "Usage: $0 [OPTIONS]"
      echo ""
      echo "Options:"
      echo "  -t, --build-type TYPE    Build type (Debug, Release, RelWithDebInfo, MinSizeRel)"
      echo "  -j, --jobs NUM           Number of parallel jobs"
      echo "  -v, --verbose           Enable verbose output"
      echo "  -c, --clean             Clean build directory before building"
      echo "  --test                  Run tests after building"
      echo "  -h, --help              Show this help message"
      echo ""
      echo "Examples:"
      echo "  $0                      # Standard release build"
      echo "  $0 --clean --test       # Clean build and run tests"
      echo "  $0 -t Debug -v          # Debug build with verbose output"
      exit 0
      ;;
    *)
      echo "Unknown option: $1"
      exit 1
      ;;
  esac
done

echo "========================================="
echo "Movidius NCS Project Build Script"
echo "========================================="
echo "Build Type: $BUILD_TYPE"
echo "Jobs: $JOBS"
echo "Clean Build: $CLEAN_BUILD"
echo "Run Tests: $RUN_TESTS"
echo "========================================="

# Function to print status messages
print_status() {
  echo -e "\033[1;34m==> $1\033[0m"
}

print_error() {
  echo -e "\033[1;31m==> ERROR: $1\033[0m"
}

print_success() {
  echo -e "\033[1;32m==> SUCCESS: $1\033[0m"
}

# Check prerequisites
print_status "Checking prerequisites..."

# Check if we're in a catkin workspace
if [[ ! -f "${PROJECT_ROOT}/.catkin_workspace" ]] && [[ ! -f "${PROJECT_ROOT}/../.catkin_workspace" ]]; then
  print_error "Not in a catkin workspace. Please initialize catkin workspace first."
  echo "Run: catkin init"
  exit 1
fi

# Check for required tools
command -v cmake >/dev/null 2>&1 || { print_error "cmake is required but not installed."; exit 1; }
command -v catkin >/dev/null 2>&1 || { print_error "catkin is required but not installed."; exit 1; }

# Check for OpenCV
pkg-config --exists opencv4 || pkg-config --exists opencv || {
  print_error "OpenCV is required but not installed."
  exit 1
}

print_success "Prerequisites check completed"

# Clean build directory if requested
if [[ $CLEAN_BUILD -eq 1 ]]; then
  print_status "Cleaning build directory..."
  rm -rf "$BUILD_DIR"
fi

# Create build directory
print_status "Creating build directory..."
mkdir -p "$BUILD_DIR"

# Configure build
print_status "Configuring build..."

cd "$BUILD_DIR"

CMAKE_ARGS=(
  -DCMAKE_BUILD_TYPE="$BUILD_TYPE"
  -DCMAKE_INSTALL_PREFIX="$INSTALL_DIR"
  -DCMAKE_EXPORT_COMPILE_COMMANDS=ON
)

if [[ $VERBOSE -eq 1 ]]; then
  CMAKE_ARGS+=(-DCMAKE_VERBOSE_MAKEFILE=ON)
fi

# Run cmake
if ! cmake "${CMAKE_ARGS[@]}" "$PROJECT_ROOT"; then
  print_error "CMake configuration failed"
  exit 1
fi

print_success "Build configuration completed"

# Build project
print_status "Building project..."

if ! make -j"$JOBS"; then
  print_error "Build failed"
  exit 1
fi

print_success "Build completed successfully"

# Install project
print_status "Installing project..."

if ! make install; then
  print_error "Installation failed"
  exit 1
fi

print_success "Installation completed"

# Run tests if requested
if [[ $RUN_TESTS -eq 1 ]]; then
  print_status "Running tests..."

  cd "$BUILD_DIR"

  if ! catkin test --no-deps --verbose; then
    print_error "Some tests failed"
    exit 1
  fi

  print_success "All tests passed"
fi

# Generate build summary
echo ""
echo "========================================="
echo "Build Summary"
echo "========================================="
echo "Build Type: $BUILD_TYPE"
echo "Build Directory: $BUILD_DIR"
echo "Install Directory: $INSTALL_DIR"
echo "Parallel Jobs: $JOBS"

if [[ $RUN_TESTS -eq 1 ]]; then
  echo "Tests: PASSED"
else
  echo "Tests: SKIPPED (use --test to run tests)"
fi

echo ""
echo "To use the installed package:"
echo "  source $INSTALL_DIR/setup.bash"
echo ""
echo "To run examples:"
echo "  roslaunch movidius_ncs_image image_classification_example.launch"
echo "========================================="

print_success "Build script completed successfully!"
