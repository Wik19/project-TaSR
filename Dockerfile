FROM ros:humble-ros-base

# 1. Set the workspace location (Matches entrypoint.sh)
WORKDIR /colcon_ws

# 2. Install build tools
RUN apt-get update && apt-get install -y \
    python3-colcon-common-extensions \
    python3-rosdep \
    git \
    ros-humble-rqt* \
    ros-humble-rviz2 \
    && rm -rf /var/lib/apt/lists/*

# 3. Copy your local source code into the container
#    (Assumes your host folder has a 'src' folder containing the driver)
COPY src /colcon_ws/src

# 4. Install dependencies
RUN . /opt/ros/humble/setup.sh && \
    apt-get update && \
    rosdep init || echo "rosdep already initialized" && \
    rosdep update && \
    rosdep install --from-paths src --ignore-src -y

# 5. Build the workspace (Note: We are already in /colcon_ws)
RUN . /opt/ros/humble/setup.sh && \
    colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release

# 6. Setup Entrypoint
COPY entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh
ENTRYPOINT ["/entrypoint.sh"]

CMD ["bash"]