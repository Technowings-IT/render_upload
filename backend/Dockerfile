# Use Ubuntu 20.04 (Focal) as base image for better compatibility with ROS 2 Jazzy
FROM ubuntu:20.04

# Set environment variables
ENV DEBIAN_FRONTEND=noninteractive \
    LANG=C.UTF-8 \
    LC_ALL=C.UTF-8 \
    ROS_DISTRO=jazzy

# Install prerequisites and ROS 2 dependencies
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
    wget \
    software-properties-common && \
    locale-gen en_US en_US.UTF-8 && \
    update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8 && \
    apt-get clean

# Add ROS 2 apt repository
RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg && \
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | tee /etc/apt/sources.list.d/ros2.list > /dev/null

# Install ROS 2 Jazzy
RUN apt-get update && apt-get install -y \
    ros-jazzy-desktop \
    python3-rosdep && \
    apt-get clean

# Initialize rosdep
RUN rosdep init && rosdep update

# Install Node.js 18 (LTS) - better choice for modern applications
RUN curl -fsSL https://deb.nodesource.com/setup_18.x | bash - && \
    apt-get install -y nodejs

# Set up working directory
WORKDIR /app

# Copy package files first for better layer caching
COPY package*.json ./

# Install npm dependencies
RUN npm install

# Copy the rest of the application code
COPY . .

# Create app directory and set permissions
RUN mkdir -p /app && chown -R 1000:1000 /app

# Expose port (adjust if your backend uses a different port)
EXPOSE 3000

# Set the default command to start the application
# This assumes your main file is app.js, adjust if different
CMD ["bash", "-c", "source /opt/ros/jazzy/setup.bash && npm start"]
