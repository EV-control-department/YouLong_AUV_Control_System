FROM osrf/ros:foxy-desktop

ARG STONEFISH_COMMIT=b21eb8e194c570ff2f61e91aeffb38d73dc25f42
ARG STONEFISH_BUILD_JOBS=1

# stonefish_ros2 is only the ROS wrapper; the Stonefish 1.6 core library is
# an external dependency and is not included in the official ROS image.
RUN apt-get update \
    && apt-get install -y --no-install-recommends \
        build-essential \
        cmake \
        git \
        libfreetype6-dev \
        libgl1-mesa-dev \
        libglu1-mesa-dev \
        libglm-dev \
        pybind11-dev \
        python3-dev \
        python3-pybind11 \
        libsdl2-dev \
        ffmpeg \
        python3-colcon-common-extensions \
        python3-numpy \
        python3-opencv \
        python3-pil \
        python3-tk \
        python3-yaml \
    && git clone https://github.com/patrykcieslak/stonefish.git /opt/stonefish \
    && git -C /opt/stonefish checkout --detach "${STONEFISH_COMMIT}" \
    && cmake -S /opt/stonefish -B /opt/stonefish/build \
        -DCMAKE_BUILD_TYPE=Release \
        -DCMAKE_INSTALL_PREFIX=/usr/local \
        -DBUILD_TESTS=OFF \
        -DEMBED_RESOURCES=OFF \
    && cmake --build /opt/stonefish/build --parallel "${STONEFISH_BUILD_JOBS}" \
    && cmake --install /opt/stonefish/build \
    && ldconfig \
    && rm -rf /opt/stonefish/.git /var/lib/apt/lists/*

# Ubuntu 20.04 ships Pillow 7, while uv_camera requires Pillow >= 9.
# Keep this in a separate layer so changing the Python dependency does not
# invalidate the Stonefish build above.
RUN apt-get update \
    && apt-get install -y --no-install-recommends python3-pip \
    && python3 -m pip install --no-cache-dir --upgrade \
        --ignore-installed \
        --target=/usr/local/lib/python3.8/dist-packages \
        'Pillow>=9.0,<11' \
    && PYTHONPATH=/usr/local/lib/python3.8/dist-packages \
       python3 -c 'import PIL; assert int(PIL.__version__.split(".")[0]) >= 9, PIL.__version__' \
    && rm -rf /var/lib/apt/lists/*

ENV PYTHONPATH=/usr/local/lib/python3.8/dist-packages

# Build both ROS overlays into the image. The repository is still bind-mounted
# at /workspace at runtime, but the compiled overlays live outside that mount
# so a fresh host needs no manual colcon step.
COPY workspace_auv /opt/youlong/src/workspace_auv
COPY workspace_sim /opt/youlong/src/workspace_sim
COPY third_party/AUV_zit6_cmake /opt/youlong/src/third_party/AUV_zit6_cmake

RUN /bin/bash -lc 'set -eo pipefail && \
    source /opt/ros/foxy/setup.bash && \
    cd /opt/youlong/src/workspace_auv && \
    CMAKE_BUILD_PARALLEL_LEVEL=1 colcon build \
        --build-base /opt/youlong/build/auv \
        --install-base /opt/youlong/install/auv \
        --parallel-workers 1 && \
    source /opt/youlong/install/auv/setup.bash && \
    cd /opt/youlong/src/workspace_sim && \
    CMAKE_BUILD_PARALLEL_LEVEL=1 colcon build \
        --build-base /opt/youlong/build/sim \
        --install-base /opt/youlong/install/sim \
        --parallel-workers 1 \
        --cmake-force-configure && \
    rm -rf /opt/youlong/src /opt/youlong/build'

ARG HOST_UID=1000
ARG HOST_GID=1000
ARG HOST_USER=dev

# Give the numeric host UID/GID a name inside the container. This keeps bind
# mounted files owned by the host user without producing "I have no name!".
RUN groupadd --gid "${HOST_GID}" "${HOST_USER}" \
    && useradd --uid "${HOST_UID}" --gid "${HOST_GID}" \
        --create-home --shell /bin/bash "${HOST_USER}"

RUN printf '%s\n' \
        'source /opt/ros/foxy/setup.bash' \
        'source /opt/youlong/install/auv/setup.bash' \
        'source /opt/youlong/install/sim/setup.bash' \
        > "/home/${HOST_USER}/.bashrc" \
    && printf '%s\n' \
        '[ -f ~/.bashrc ] && . ~/.bashrc' \
        > "/home/${HOST_USER}/.bash_profile" \
    && chown "${HOST_UID}:${HOST_GID}" \
        "/home/${HOST_USER}/.bashrc" "/home/${HOST_USER}/.bash_profile"

ENV HOME=/home/${HOST_USER}
USER ${HOST_USER}
WORKDIR /workspace
