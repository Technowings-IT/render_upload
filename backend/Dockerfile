# Use ROS Jazzy base image
FROM osrf/ros:jazzy-desktop

# Install Node.js 20
RUN apt-get update && apt-get install -y curl gnupg2 \
  && curl -fsSL https://deb.nodesource.com/setup_20.x | bash - \
  && apt-get install -y nodejs build-essential \
  && apt-get clean

# Create app directory
WORKDIR /app

# Copy backend files
COPY . .

# Optional: remove lock file to avoid EINTEGRITY
RUN rm -f package-lock.json

# Install dependencies
RUN npm install

# Set default command to run the backend
CMD [ "node", "app.js" ]
