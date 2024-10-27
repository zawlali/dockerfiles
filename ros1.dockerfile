# Use the ROS Melodic base image
FROM ros:melodic-ros-base

# Install curl
RUN apt-get update && apt-get install -y curl git ros-melodic-foxglove-bridge && rm -rf /var/lib/apt/lists/*

# Install Rust
RUN curl --proto '=https' --tlsv1.2 -sSf https://sh.rustup.rs | sh -s -- -y

# Add Rust binaries to PATH
ENV PATH="/root/.cargo/bin:${PATH}"

# Set the working directory
WORKDIR /root

# Clone the repository with recursive flag
RUN git clone https://github.com/eclipse-zenoh/zenoh-plugin-ros1.git --recursive

# Change to the cloned directory
WORKDIR /root/zenoh-plugin-ros1

# Update Rust and build the project
RUN rustup update && cargo build --release

# Install Tailscale
RUN curl -fsSL https://tailscale.com/install.sh | sh && \
    rm -rf /var/lib/apt/lists/*

# Install jsk_recognition_msgs
RUN apt-get update && \
    apt-get install -y ros-melodic-jsk-recognition-msgs && \
    rm -rf /var/lib/apt/lists/*

# Create workspace for Autoware AI messages
RUN mkdir -p /root/autowareai_msgs_ws/src && \
    cd /root/autowareai_msgs_ws/src && \
    git clone https://github.com/autowarefoundation/autoware_ai_messages.git && \
    cd /root/autowareai_msgs_ws && \
    . /opt/ros/melodic/setup.sh && \
    catkin_make

# Source the Autoware AI messages workspace
RUN echo "source /root/autowareai_msgs_ws/devel/setup.bash" >> /root/.bashrc

# Create the startup script
RUN echo '#!/bin/bash' > /start_bridges.sh && \
    echo '' >> /start_bridges.sh && \
    echo 'source /opt/ros/melodic/setup.bash' >> /start_bridges.sh && \
    echo 'source /root/autowareai_msgs_ws/devel/setup.bash' >> /start_bridges.sh && \
    echo '' >> /start_bridges.sh && \
    echo 'echo "Starting Tailscale..."' >> /start_bridges.sh && \
    echo 'tailscaled --state=mem: &' >> /start_bridges.sh && \
    echo 'sleep 5' >> /start_bridges.sh && \
    echo 'tailscale up --authkey=$TAILSCALE_AUTHKEY' >> /start_bridges.sh && \
    echo '' >> /start_bridges.sh && \
    echo 'echo "Starting Zenoh bridge and Foxglove bridge in the foreground..."' >> /start_bridges.sh && \
    echo '~/zenoh-plugin-ros1/target/release/./zenoh-bridge-ros1 -e tcp/$CONNECT_IP:7447  --with_rosmaster true &' >> /start_bridges.sh && \
    echo 'rosrun foxglove_bridge foxglove_bridge' >> /start_bridges.sh && \
    chmod +x /start_bridges.sh

# Add a command to .bashrc to run the script
RUN echo 'echo "To start the bridges, run: /start_bridges.sh"' >> /root/.bashrc


# Set the default command (you can override this when running the container)
CMD ["/bin/bash"]
