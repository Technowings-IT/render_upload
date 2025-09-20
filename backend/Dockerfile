# Use ROS 2 Jazzy base image
FROM osrf/ros:jazzy-desktop

# Install Node.js 16 (LTS)
RUN apt-get update && apt-get install -y curl gnupg2 \
  && curl -fsSL https://deb.nodesource.com/setup_16.x | bash - \
  && apt-get install -y nodejs build-essential \
  && apt-get clean

# Set working directory inside container
WORKDIR /app

# Copy backend code into container
COPY . .

# Install Node.js dependencies
RUN npm install

# Source ROS 2 setup and run your Node.js backend
CMD [ "bash", "-c", "source /opt/ros/jazzy/setup.bash && node app.js" ]

# Expose port your backend listens on (adjust if needed)
EXPOSE 3000


