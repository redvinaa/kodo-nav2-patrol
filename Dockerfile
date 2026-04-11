FROM osrf/ros:humble-desktop-full

ENV DEBIAN_FRONTEND=noninteractive

# Install tools
RUN apt-get update && apt-get install -y --no-install-recommends \
    python3-colcon-common-extensions \
    python3-rosdep \
    python3-vcstool \
    && rm -rf /var/lib/apt/lists/*

# Initialize rosdep early
RUN rosdep init || true && \
    rosdep update

WORKDIR /ros2_ws

# Copy source to get deps for rosdep
# TODO: Copy only package.xml
COPY src/ src/

# Install dependencies
RUN apt-get update && \
    . /opt/ros/humble/setup.sh && \
    rosdep install --from-paths src --ignore-src -y --rosdistro humble && \
    rm -rf /var/lib/apt/lists/*

# # Setup environment script
# COPY setup_env.sh /ros2_ws/setup_env.sh
# RUN chmod +x /ros2_ws/setup_env.sh

# Source setup on every bash login
RUN echo "source /ros2_ws/setup_env.bash" >> /root/.bashrc

CMD ["bash"]
