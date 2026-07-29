FROM microros/micro-ros-agent:jazzy

USER root

# Install colcon and ROS 2 message generation tools
RUN apt-get update && apt-get install -y \
    python3-colcon-common-extensions \
    build-essential \
    ros-jazzy-rosidl-default-generators \
    ros-jazzy-rosidl-default-runtime \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /uros_ws

COPY firmware/extra_packages/ferrari_vehicle src/ferrari_vehicle

RUN /bin/bash -c "source /opt/ros/jazzy/setup.bash && colcon build"

ENTRYPOINT ["/bin/bash", "-c", "source /opt/ros/jazzy/setup.bash && source install/setup.bash && ros2 run micro_ros_agent micro_ros_agent \"$@\"", "--"]