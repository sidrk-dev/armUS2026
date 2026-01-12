FROM ros:humble-desktop

# Install essential build tools and dependencies
RUN apt-get update && apt-get install -y \
    python3-pip \
    python3-serial \
    can-utils \
    usbutils \
    ros-humble-moveit \
    ros-humble-ros2-control \
    ros-humble-ros2-controllers \
    ros-humble-xacro \
    ros-humble-joint-state-publisher-gui \
    ros-humble-robot-state-publisher \
    # X11 and GUI support
    libgl1-mesa-glx \
    libgl1-mesa-dri \
    mesa-utils \
    x11-apps \
    # Development tools
    vim \
    nano \
    htop \
    && rm -rf /var/lib/apt/lists/*

# Install python dependencies
RUN pip3 install pyserial

# Create workspace directory
WORKDIR /root/ws_karura

# Set up workspace structure
RUN mkdir -p /root/ws_karura/src

# Add useful aliases to bashrc
RUN echo "" >> /root/.bashrc && \
    echo "# ROS2 setup" >> /root/.bashrc && \
    echo "source /opt/ros/humble/setup.bash" >> /root/.bashrc && \
    echo "" >> /root/.bashrc && \
    echo "# Workspace setup (if built)" >> /root/.bashrc && \
    echo "if [ -f /root/ws_karura/install/setup.bash ]; then" >> /root/.bashrc && \
    echo "    source /root/ws_karura/install/setup.bash" >> /root/.bashrc && \
    echo "fi" >> /root/.bashrc && \
    echo "" >> /root/.bashrc && \
    echo "# Useful aliases" >> /root/.bashrc && \
    echo "alias cb='cd /root/ws_karura && colcon build --symlink-install'" >> /root/.bashrc && \
    echo "alias moveit='ros2 launch karura_arm_moveit_config demo.launch.py'" >> /root/.bashrc && \
    echo "alias serial='ros2 run serial_comm Arm_serial'" >> /root/.bashrc && \
    echo "alias joints='ros2 run send_target_joint_degrees send_target_joint_degrees'" >> /root/.bashrc && \
    echo "" >> /root/.bashrc && \
    echo "cd /root/ws_karura" >> /root/.bashrc

# Entrypoint
ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["bash"]
