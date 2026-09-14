FROM ros:noetic-ros-core

ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update \
    && apt-get install -y --no-install-recommends \
        ros-noetic-turtlesim \
        ros-noetic-rqt-graph \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /root
ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["bash"]
