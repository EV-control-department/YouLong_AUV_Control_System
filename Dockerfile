ARG ROS_DISTRO=foxy
FROM osrf/ros:${ROS_DISTRO}-desktop

ARG ROS_DISTRO
ENV ROS_DISTRO=${ROS_DISTRO}
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
        libeigen3-dev \
        libpcl-dev \
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

# Ubuntu 20.04 ships Pillow 7, while uv_camera requires Pillow >= 9.  The
# same image also needs PySide6 for the uv_log player and visualization tools.
# PySide6 6.5 dropped Python 3.8 support, so Foxy must use the last compatible
# 6.2.x release while Jazzy can use the newer series.
# Keep this in a separate layer so changing the Python dependency does not
# invalidate the Stonefish build above.
RUN apt-get update \
    && apt-get install -y --no-install-recommends python3-pip \
    && if [ "$(python3 -c 'import sys; print(f"{sys.version_info.major}.{sys.version_info.minor}")')" = "3.8" ]; then \
           PYSIDE6_SPEC="PySide6>=6.2,<6.3"; \
       else \
           PYSIDE6_SPEC="PySide6>=6.5,<7"; \
       fi \
    && python3 -m pip install --no-cache-dir --upgrade \
        --ignore-installed \
        --target="/usr/local/lib/python$(python3 -c 'import sys; print(f"{sys.version_info.major}.{sys.version_info.minor}")')/dist-packages" \
        'Pillow>=9.0,<11' \
        "${PYSIDE6_SPEC}" \
    && PYTHONPATH="/usr/local/lib/python$(python3 -c 'import sys; print(f"{sys.version_info.major}.{sys.version_info.minor}")')/dist-packages" \
       python3 -c 'import PIL; assert int(PIL.__version__.split(".")[0]) >= 9, PIL.__version__; import PySide6; print(PySide6.__version__)' \
    && rm -rf /var/lib/apt/lists/*

# Keep both supported interpreter paths available. Only one exists in a
# concrete image, and this avoids a second distro-specific Dockerfile.
ENV PYTHONPATH=/usr/local/lib/python3.8/dist-packages:/usr/local/lib/python3.12/dist-packages

# Build both ROS overlays into the image. The repository is still bind-mounted
# at /workspace at runtime, but the compiled overlays live outside that mount
# so a fresh host needs no manual colcon step.
COPY workspace_auv /opt/youlong/src/workspace_auv
COPY workspace_sim /opt/youlong/src/workspace_sim
COPY third_party/AUV_zit6_cmake /opt/youlong/src/third_party/AUV_zit6_cmake

RUN /bin/bash -lc 'set -eo pipefail && \
    source /opt/ros/${ROS_DISTRO}/setup.bash && \
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

# The GUI renders Chinese labels and uses pygame for gamepad input.  Keep the
# font packages after the native build layers so changing the GUI environment
# does not trigger another Stonefish or ROS workspace rebuild.
RUN apt-get update \
    && apt-get install -y --no-install-recommends \
        fontconfig \
        fonts-dejavu-core \
        fonts-liberation \
        fonts-noto-cjk \
        fonts-noto-core \
        fonts-noto-color-emoji \
        fonts-noto-mono \
        locales \
        python3-pygame \
    && locale-gen en_US.UTF-8 zh_CN.UTF-8 \
    && fc-cache -f -v \
    && test "$(fc-match -f '%{family}' 'sans-serif:lang=zh-cn' | head -n 1)" = "Noto Sans CJK SC" \
    && rm -rf /var/lib/apt/lists/*

# ROS's rqt is a Qt/X11 application.  Explicitly select the X11 backend and
# add the CJK directory to Qt's font search path so it does not depend on the
# host font configuration mounted into the container.
ENV QT_QPA_PLATFORM=xcb \
    QT_QPA_FONTDIR=/usr/share/fonts/opentype/noto \
    QT_X11_NO_MITSHM=1

ARG HOST_UID=1000
ARG HOST_GID=1000
ARG HOST_USER=dev

# Give the numeric host UID/GID a name inside the container. The ROS base
# image may already contain the requested numeric group (commonly GID 1000),
# so reuse it instead of unconditionally trying to create a duplicate group.
# Likewise, allow a second login name for an already-used UID: the numeric UID
# is what controls ownership of bind-mounted files, while HOST_USER is needed
# by Compose's `user:` setting and by the shell environment below.
RUN set -eux; \
    if ! getent group "${HOST_GID}" >/dev/null; then \
        groupadd --gid "${HOST_GID}" "${HOST_USER}"; \
    fi; \
    if getent passwd "${HOST_USER}" >/dev/null; then \
        test "$(id -u "${HOST_USER}")" = "${HOST_UID}"; \
    else \
        useradd --uid "${HOST_UID}" --non-unique --gid "${HOST_GID}" \
            --create-home --shell /bin/bash "${HOST_USER}"; \
    fi

RUN printf '%s\n' \
        'source /opt/ros/${ROS_DISTRO}/setup.bash' \
        'source /opt/youlong/install/auv/setup.bash' \
        'source /opt/youlong/install/sim/setup.bash' \
        'alias uuv_src="source /workspace/install/setup.bash"' \
        'cd /workspace' \
        > "/home/${HOST_USER}/.bashrc" \
    && printf '%s\n' \
        '[ -f ~/.bashrc ] && . ~/.bashrc' \
        > "/home/${HOST_USER}/.bash_profile" \
    && chown "${HOST_UID}:${HOST_GID}" \
        "/home/${HOST_USER}/.bashrc" "/home/${HOST_USER}/.bash_profile"

ENV HOME=/home/${HOST_USER}
USER ${HOST_USER}
WORKDIR /workspace
