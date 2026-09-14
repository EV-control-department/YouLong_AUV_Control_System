FROM ros:jazzy-ros-core

SHELL ["/bin/bash", "-c"]

RUN apt-get update \
    && apt-get install -y --no-install-recommends \
        python3-colcon-common-extensions \
        ros-jazzy-ros-gz-sim \
        ros-jazzy-ros-gz-bridge \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /smartcar_ws
ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["bash"]
