# Use Ubuntu 18.04 (Bionic) as Jazzy is targeted here
FROM ubuntu:18.04

ENV DEBIAN_FRONTEND=noninteractive \
    LANG=C.UTF-8 \
    LC_ALL=C.UTF-8 \
    ROS_DISTRO=jazzy \
    PATH=/opt/ros/jazzy/bin:$PATH \
    LD_LIBRARY_PATH=/opt/ros/jazzy/lib:$LD_LIBRARY_PATH \
    PYTHONPATH=/opt/ros/jazzy/lib/python3.6/site-packages:$PYTHONPATH

# Install prerequisites, curl, wget, gnupg, lsb-release, locales, python3, build tools
RUN apt-get update && apt-get install -y \
    locales \
    curl \
    gnupg2 \
    lsb-release \
    build-essential \
    python3-pip \
    python3-setuptools \
    python3-colcon-common-extensions \
    python3-rosdep \
    python3-vcstool \
    git \
    wget && \
    locale-gen C.UTF-8

# Add ROS2 Jazzy apt repo and key (archived)
RUN curl -sSL http://repo.ros2.org/repos.key | apt-key add - && \
    echo "deb http://repo.ros2.org/ubuntu/main bionic main" > /etc/apt/sources.list.d/ros2-latest.list && \
    apt-get update && \
    apt-get install -y ros-jazzy-desktop

# Initialize rosdep (required for ros package dependencies)
RUN rosdep init && rosdep update

# Install Node.js 14 (compatible with jazzy/rclnodejs)
RUN curl -sL https://deb.nodesource.com/setup_14.x | bash - && \
    apt-get install -y nodejs && \
    npm install -g npm@8

# Setup working directory
WORKDIR /app

# Copy package files & install npm dependencies
COPY package*.json ./

# Install rclnodejs dependencies and rebuild native addons
RUN npm install --build-from-source

# Copy app source code
COPY . .

# Expose port (customize if needed)
EXPOSE 3000

# Source ROS setup and start app
CMD ["/bin/bash", "-c", "source /opt/ros/jazzy/setup.bash && npm start"]
