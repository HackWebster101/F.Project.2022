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

FROM ros:melodic-ros-core

# Set environment variables
ENV DEBIAN_FRONTEND=noninteractive
ENV ROS_DISTRO=melodic

# Install system dependencies
RUN apt-get update && apt-get install -y \
    build-essential \
    cmake \
    git \
    wget \
    unzip \
    python-catkin-tools \
    python-rosdep \
    python-wstool \
    ros-${ROS_DISTRO}-catkin \
    && rm -rf /var/lib/apt/lists/*

# Install OpenCV
RUN apt-get update && apt-get install -y \
    libopencv-dev \
    && rm -rf /var/lib/apt/lists/*

# Install Boost
RUN apt-get update && apt-get install -y \
    libboost-all-dev \
    && rm -rf /var/lib/apt/lists/*

# Install Intel Movidius NCS SDK
RUN mkdir -p /opt/movidius && \
    cd /opt/movidius && \
    wget -O ncsdk.tar.gz "https://github.com/movidius/ncsdk/releases/download/v2.10.01.01/ncsdk-2.10.01.01.tar.gz" && \
    tar -xzf ncsdk.tar.gz && \
    cd ncsdk-2.10.01.01 && \
    make install && \
    cd / && rm -rf /opt/movidius/ncsdk.tar.gz /opt/movidius/ncsdk-2.10.01.01

# Set up workspace
RUN mkdir -p /catkin_ws/src
WORKDIR /catkin_ws/src

# Copy project files
COPY . /catkin_ws/src/movidius_ncs_project/

# Install ROS dependencies
RUN cd /catkin_ws && \
    rosdep init && \
    rosdep update && \
    rosdep install --from-paths src --ignore-src -r -y

# Build the project
RUN cd /catkin_ws && \
    catkin config --extend /opt/ros/$ROS_DISTRO && \
    catkin build movidius_ncs_project

# Set up entrypoint
RUN echo "source /catkin_ws/devel/setup.bash" >> ~/.bashrc

# Create data directories
RUN mkdir -p /catkin_ws/data/images /catkin_ws/data/models /catkin_ws/data/labels /catkin_ws/data/results

# Copy sample data
COPY data/images/* /catkin_ws/data/images/
COPY data/labels/* /catkin_ws/data/labels/

# Set working directory
WORKDIR /catkin_ws

# Default command
CMD ["/bin/bash"]
