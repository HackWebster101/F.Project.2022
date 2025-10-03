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

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

# Function to print colored output
print_info() {
  echo -e "${GREEN}[INFO]${NC} $1"
}

print_warn() {
  echo -e "${YELLOW}[WARN]${NC} $1"
}

print_error() {
  echo -e "${RED}[ERROR]${NC} $1"
}

print_header() {
  echo -e "${GREEN}=========================================${NC}"
  echo -e "${GREEN}$1${NC}"
  echo -e "${GREEN}=========================================${NC}"
}

# Check if running as root (some operations need sudo)
if [[ $EUID -eq 0 ]]; then
  print_warn "Running as root. This may cause permission issues later."
fi

print_header "Movidius NCS Project Setup"

# Check system requirements
print_info "Checking system requirements..."

# Check Ubuntu version
UBUNTU_VERSION=$(lsb_release -rs 2>/dev/null || echo "unknown")
if [[ ! "$UBUNTU_VERSION" =~ ^(16\.04|18\.04|20\.04|22\.04)$ ]]; then
  print_warn "Untested Ubuntu version: $UBUNTU_VERSION"
  print_warn "Supported versions: 16.04, 18.04, 20.04, 22.04"
fi

# Check architecture
ARCH=$(uname -m)
if [[ "$ARCH" != "x86_64" ]]; then
  print_error "Unsupported architecture: $ARCH"
  print_error "Only x86_64 is supported"
  exit 1
fi

print_info "System check completed"

# Install system packages
print_header "Installing System Dependencies"

PACKAGES=(
  build-essential
  cmake
  git
  wget
  unzip
  python-catkin-tools
  python-rosdep
  python-wstool
  libopencv-dev
  libboost-all-dev
)

print_info "Installing packages: ${PACKAGES[*]}"

sudo apt-get update
sudo apt-get install -y "${PACKAGES[@]}" || {
  print_error "Failed to install system packages"
  exit 1
}

print_info "System packages installed successfully"

# Install ROS
print_header "Setting up ROS"

ROS_DISTRO="melodic"
if [[ "$UBUNTU_VERSION" =~ ^(20\.04|22\.04)$ ]]; then
  ROS_DISTRO="noetic"
fi

print_info "Installing ROS $ROS_DISTRO..."

# Check if ROS is already installed
if ! dpkg -l | grep -q "ros-$ROS_DISTRO"; then
  sudo sh -c "echo \"deb http://packages.ros.org/ros/ubuntu \$(lsb_release -sc) main\" > /etc/apt/sources.list.d/ros-latest.list"
  sudo apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654

  sudo apt-get update
  sudo apt-get install -y "ros-$ROS_DISTRO-ros-base"

  # Initialize rosdep
  sudo rosdep init
  rosdep update

  # Add ROS setup to bashrc
  echo "source /opt/ros/$ROS_DISTRO/setup.bash" >> ~/.bashrc
else
  print_info "ROS $ROS_DISTRO already installed"
fi

print_info "ROS setup completed"

# Install Intel Movidius NCSDK
print_header "Installing Intel Movidius NCSDK"

NCSDK_VERSION="2.10.01.01"
NCSDK_URL="https://github.com/movidius/ncsdk/releases/download/v$NCSDK_VERSION/ncsdk-$NCSDK_VERSION.tar.gz"

print_info "Installing NCSDK $NCSDK_VERSION..."

if [[ ! -d "/opt/movidius/ncsdk-$NCSDK_VERSION" ]]; then
  cd /tmp
  wget -O ncsdk.tar.gz "$NCSDK_URL"
  tar -xzf ncsdk.tar.gz

  cd "ncsdk-$NCSDK_VERSION"
  sudo make install

  cd /
  rm -rf /tmp/ncsdk.tar.gz /tmp/ncsdk-$NCSDK_VERSION

  # Add NCSDK to PATH
  echo 'export PATH=/opt/movidius/nc-sdk/bin:$PATH' >> ~/.bashrc
  echo 'export LD_LIBRARY_PATH=/opt/movidius/nc-sdk/lib:$LD_LIBRARY_PATH' >> ~/.bashrc
else
  print_info "NCSDK already installed"
fi

print_info "NCSDK installation completed"

# Set up catkin workspace
print_header "Setting up Catkin Workspace"

CATKIN_WS="$HOME/catkin_ws"

if [[ ! -d "$CATKIN_WS/src" ]]; then
  mkdir -p "$CATKIN_WS/src"
fi

cd "$CATKIN_WS"

# Initialize catkin workspace if not already done
if [[ ! -f ".catkin_workspace" ]]; then
  catkin init
fi

# Configure catkin workspace
catkin config --extend "/opt/ros/$ROS_DISTRO"
catkin config --cmake-args -DCMAKE_BUILD_TYPE=Release

print_info "Catkin workspace configured"

# Install project dependencies
print_header "Installing Project Dependencies"

cd "$CATKIN_WS"

# Install ROS dependencies
print_info "Installing ROS dependencies..."
rosdep install --from-paths src --ignore-src -r -y || {
  print_warn "Some ROS dependencies may be missing"
}

print_info "Dependencies installation completed"

# Create data directories
print_header "Creating Data Directories"

mkdir -p "$CATKIN_WS/data/images"
mkdir -p "$CATKIN_WS/data/models"
mkdir -p "$CATKIN_WS/data/labels"
mkdir -p "$CATKIN_WS/data/results"
mkdir -p "$CATKIN_WS/logs"

print_info "Data directories created"

# Set up udev rules for NCS devices
print_header "Setting up Device Permissions"

UDEV_RULES="/etc/udev/rules.d/97-movidius-ncs.rules"

if [[ ! -f "$UDEV_RULES" ]]; then
  print_info "Setting up udev rules for NCS devices..."

  sudo tee "$UDEV_RULES" > /dev/null << EOF
# Intel Movidius Neural Compute Stick
SUBSYSTEM=="usb", ATTRS{idVendor}=="03e7", ATTRS{idProduct}=="f63b", MODE="0666", GROUP="plugdev"
SUBSYSTEM=="usb", ATTRS{idVendor}=="03e7", ATTRS{idProduct}=="2150", MODE="0666", GROUP="plugdev"
EOF

  sudo udevadm control --reload-rules
  sudo udevadm trigger

  print_info "Udev rules configured"
else
  print_info "Udev rules already exist"
fi

# Add user to plugdev group
if ! groups | grep -q plugdev; then
  print_info "Adding user to plugdev group..."
  sudo usermod -a -G plugdev $USER
  print_warn "Please logout and login again for group changes to take effect"
fi

print_header "Setup Completed Successfully!"

echo ""
echo "Next steps:"
echo "1. Logout and login again to apply group changes"
echo "2. Source ROS environment:"
echo "   source /opt/ros/$ROS_DISTRO/setup.bash"
echo "3. Build the project:"
echo "   cd $CATKIN_WS"
echo "   catkin build movidius_ncs_project"
echo "4. Test installation:"
echo "   roslaunch movidius_ncs_image image_classification_example.launch"
echo ""
echo "For Docker usage:"
echo "   docker build -t movidius-ncs:latest ."
echo "   docker run -it --device=/dev/bus/usb:/dev/bus/usb movidius-ncs:latest"
echo ""

print_success "Setup completed! Happy deep learning with NCS! 🤖🧠"
