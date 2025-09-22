# Use Ubuntu 18.04 (Bionic) as Jazzy is targeted here
FROM ubuntu:18.04

ENV DEBIAN_FRONTEND=noninteractive \
    LANG=C.UTF-8 \
    LC_ALL=C.UTF-8 \
    ROS_DISTRO=jazzy \
    PATH=/opt/ros/jazzy/bin:$PATH \
    LD_LIBRARY_PATH=/opt/ros/jazzy/lib:$LD_LIBRARY_PATH \
    PYTHONPATH=/opt/ros/jazzy/lib/python3.6/site-packages:$PYTHONPATH

# Enable universe repo and install basic tools
RUN apt-get update && apt-get install -y software-properties-common && \
    add-apt-repository universe && apt-get update

# Install OS-level dependencies
RUN apt-get install -y --no-install-recommends \
    locales \
    curl \
    gnupg2 \
    lsb-release \
    build-essential \
    python3-pip \
    python3-setuptools \
    git \
    wget && \
    locale-gen C.UTF-8 && \
    rm -rf /var/lib/apt/lists/*

# Install Python-based ROS tools via pip
RUN pip3 install -U \
    colcon-common-extensions \
    rosdep \
    vcstool

# Add ROS2 Jazzy apt repo and install
RUN curl -sSL http://repo.ros2.org/repos.key | apt-key add - && \
    echo "deb http://repo.ros2.org/ubuntu/main bionic main" > /etc/apt/sources.list.d/ros2-latest.list && \
    apt-get update && \
    apt-get install -y --no-install-recommends ros-jazzy-desktop && \
    rm -rf /var/lib/apt/lists/*

# Initialize rosdep (required for ros package dependencies)
RUN rosdep init && rosdep update

# Install Node.js 14 (compatible with jazzy/rclnodejs)
RUN curl -sL https://deb.nodesource.com/setup_14.x | bash - && \
    apt-get install -y --no-install-recommends nodejs && \
    npm install -g npm@8 && \
    rm -rf /var/lib/apt/lists/*

# Set working directory
WORKDIR /app

# Copy package files & install npm dependencies
COPY package*.json ./

RUN npm install --build-from-source

# Copy app source
COPY . .

# Expose app port
EXPOSE 3000

# Run app
CMD ["/bin/bash", "-c", "source /opt/ros/jazzy/setup.bash && npm start"]
