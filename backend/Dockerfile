# Use Ubuntu 24.04 (Noble) as ROS2 Jazzy officially requires this version
FROM ubuntu:24.04

ENV DEBIAN_FRONTEND=noninteractive \
    LANG=C.UTF-8 \
    LC_ALL=C.UTF-8 \
    ROS_DISTRO=jazzy \
    PATH=/opt/ros/jazzy/bin:$PATH \
    LD_LIBRARY_PATH=/opt/ros/jazzy/lib:$LD_LIBRARY_PATH \
    PYTHONPATH=/opt/ros/jazzy/lib/python3.12/site-packages:$PYTHONPATH

# Enable universe repo and install basic tools
RUN apt-get update && apt-get install -y software-properties-common && \
    add-apt-repository universe && apt-get update

# Install OS-level dependencies and configure locale
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
    echo "C.UTF-8 UTF-8" >> /etc/locale.gen && \
    locale-gen C.UTF-8 && \
    update-locale LANG=C.UTF-8 LC_ALL=C.UTF-8 && \
    rm -rf /var/lib/apt/lists/*

# Install Python-based ROS tools via pip
RUN pip3 install -U \
    colcon-common-extensions \
    rosdep \
    vcstool

# Add ROS2 Jazzy apt repo and install
RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg && \
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | tee /etc/apt/sources.list.d/ros2.list > /dev/null && \
    apt-get update && \
    apt-get install -y --no-install-recommends ros-jazzy-desktop && \
    rm -rf /var/lib/apt/lists/*

# Initialize rosdep (required for ros package dependencies)
RUN rosdep init && rosdep update

# Install Node.js 18 (recommended for modern ROS2 development)
RUN curl -fsSL https://deb.nodesource.com/setup_18.x | bash - && \
    apt-get install -y --no-install-recommends nodejs && \
    npm install -g npm@latest && \
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