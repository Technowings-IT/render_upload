FROM osrf/ros:jazzy-desktop

# Install Node.js 20 (LTS) + build tools
RUN apt-get update && \
    apt-get install -y curl gnupg2 build-essential python3 git && \
    curl -fsSL https://deb.nodesource.com/setup_20.x | bash - && \
    apt-get install -y nodejs && \
    apt-get clean

# Show Node/NPM versions
RUN node -v && npm -v

# Set working directory
WORKDIR /app

# Copy project files
COPY . .

# Install dependencies
RUN npm install

# Optional: Expose a port (if you’re running a server)
# EXPOSE 3000

# Default command
CMD ["npm", "start", "node"]
