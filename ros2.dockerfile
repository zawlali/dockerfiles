# This is an auto generated Dockerfile for ros:ros-base
# generated from docker_images_ros2/create_ros_image.Dockerfile.em  
FROM ros:humble-ros-core-jammy

# install bootstrap tools
RUN apt-get update && apt-get install --no-install-recommends -y \
    build-essential \
    git \
    python3-colcon-common-extensions \
    python3-colcon-mixin \
    python3-rosdep \
    python3-vcstool \
    && rm -rf /var/lib/apt/lists/*

# bootstrap rosdep
RUN rosdep init && \
  rosdep update --rosdistro $ROS_DISTRO

# setup colcon mixin and metadata
RUN colcon mixin add default \
      https://raw.githubusercontent.com/colcon/colcon-mixin-repository/master/index.yaml && \
    colcon mixin update && \
    colcon metadata add default \
      https://raw.githubusercontent.com/colcon/colcon-metadata-repository/master/index.yaml && \
    colcon metadata update

# install ros2 packages
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-humble-ros-base=0.10.0-1* \
    ros-humble-foxglove-bridge \
    ros-humble-geographic-msgs \
    && rm -rf /var/lib/apt/lists/*

# Clone and build autoware_msgs
RUN mkdir -p /autoware_msgs_ws/src && \
    cd /autoware_msgs_ws/src && \
    git clone https://github.com/autowarefoundation/autoware_msgs.git && \
    cd /autoware_msgs_ws && \
    . /opt/ros/humble/setup.sh && \
    colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release

# Clone and build autoware_auto_msgs
RUN mkdir -p /autoware_auto_msgs_ws/src && \
    cd /autoware_auto_msgs_ws/src && \
    git clone https://github.com/tier4/autoware_auto_msgs.git && \
    cd /autoware_auto_msgs_ws && \
    . /opt/ros/humble/setup.sh && \
    colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release

# Clone and build autoware_adapi_msgs
RUN mkdir -p /autoware_adapi_msgs_ws/src && \
    cd /autoware_adapi_msgs_ws/src && \
    git clone https://github.com/autowarefoundation/autoware_adapi_msgs.git && \
    cd /autoware_adapi_msgs_ws && \
    . /opt/ros/humble/setup.sh && \
    rosdep install -y --from-paths src --ignore-src --rosdistro $ROS_DISTRO && \
    colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release && \
    rm -rf /var/lib/apt/lists/*

# Source the workspaces in .bashrc
RUN echo "source /autoware_msgs_ws/install/setup.bash" >> /root/.bashrc && \
    echo "source /autoware_auto_msgs_ws/install/setup.bash" >> /root/.bashrc && \
    echo "source /autoware_adapi_msgs_ws/install/setup.bash" >> /root/.bashrc


# Add Zenoh repository and install zenoh-bridge-ros2dds
RUN echo "deb [trusted=yes] https://download.eclipse.org/zenoh/debian-repo/ /" | tee -a /etc/apt/sources.list > /dev/null && \
    apt-get update && \
    apt-get install -y zenoh-bridge-ros2dds && \
    rm -rf /var/lib/apt/lists/*

# Install curl and Tailscale
RUN apt-get update && apt-get install -y curl && \
    curl -fsSL https://tailscale.com/install.sh | sh && \
    rm -rf /var/lib/apt/lists/*

# Create the startup script
RUN echo '#!/bin/bash' > /start_bridges.sh && \
    echo '' >> /start_bridges.sh && \
    echo 'source /opt/ros/humble/setup.bash' >> /start_bridges.sh && \
    echo 'source /autoware_msgs_ws/install/setup.bash' >> /start_bridges.sh && \
    echo 'source /autoware_auto_msgs_ws/install/setup.bash' >> /start_bridges.sh && \
    echo 'source /autoware_adapi_msgs_ws/install/setup.bash' >> /start_bridges.sh && \
    echo '' >> /start_bridges.sh && \
    echo 'echo "Starting Tailscale..."' >> /start_bridges.sh && \
    echo 'tailscaled --state=mem: &' >> /start_bridges.sh && \
    echo 'sleep 5' >> /start_bridges.sh && \
    echo 'tailscale up --authkey=$TAILSCALE_AUTHKEY' >> /start_bridges.sh && \
    echo '' >> /start_bridges.sh && \
    echo 'echo "Starting Zenoh bridge and Foxglove bridge in the foreground..."' >> /start_bridges.sh && \
    echo 'zenoh-bridge-ros2dds -e tcp/$CONNECT_IP:7447 &' >> /start_bridges.sh && \
    echo 'ros2 run foxglove_bridge foxglove_bridge' >> /start_bridges.sh && \
    chmod +x /start_bridges.sh

# Add a command to .bashrc to run the script
RUN echo 'echo "To start the bridges, run: /start_bridges.sh"' >> /root/.bashrc

# Set the default command to bash
CMD ["/bin/bash"]
