FROM osrf/ros:humble-desktop-full

# Install dependencies
RUN apt-get update -y && \
    apt-get install -y curl python3-pip && \
    rm -rf /var/lib/apt/lists/*

# Set up workspace
WORKDIR /workspace

# Copy necessary files
COPY . .

# Install ROS dependencies

RUN rosdep install --from-paths . --ignore-src -ry || true

# Build the project
RUN . /opt/ros/humble/setup.sh && colcon build

# Source the setup script
RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc

# Run bash shell
CMD ["/bin/bash"]