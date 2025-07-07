FROM ros:noetic

ENV DEBIAN_FRONTEND=noninteractive
ENV ROS_WORKSPACE=/catkin_ws

# Install dependencies
RUN apt-get update && apt-get install -y \
    python3-pip \
    python3-rosdep \
    python3-catkin-tools \
    ros-noetic-geometry-msgs \
    && rm -rf /var/lib/apt/lists/*

# Initialize rosdep
RUN rosdep init || true
RUN rosdep update

# Copy source files
WORKDIR /
COPY . /catkin_ws/src/nissanradar

# Make scripts executable
RUN chmod +x /catkin_ws/src/nissanradar/src/*.py

# Build workspace
RUN /bin/bash -c "source /opt/ros/noetic/setup.bash && \
    cd /catkin_ws && \
    rosdep install --from-paths src --ignore-src -r -y && \
    catkin_make"

# Source environment on shell start
RUN echo 'source /opt/ros/noetic/setup.bash' >> ~/.bashrc && \
    echo 'source /catkin_ws/devel/setup.bash' >> ~/.bashrc

COPY entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh
ENTRYPOINT ["/entrypoint.sh"]
CMD ["display:=number"]
