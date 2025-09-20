FROM osrf/ros:jazzy-desktop

# Install Node.js 18 and dependencies
RUN apt-get update && apt-get install -y \
    curl gnupg2 python3-dev build-essential cmake \
 && curl -fsSL https://deb.nodesource.com/setup_18.x | bash - \
 && apt-get install -y nodejs \
 && apt-get clean

WORKDIR /app
COPY . .

# Source ROS and install npm dependencies
RUN . /opt/ros/jazzy/setup.sh && npm install

CMD ["npm", "start"]
