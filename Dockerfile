ARG YOULONG_ROS_DISTRO=foxy
FROM osrf/ros:${YOULONG_ROS_DISTRO}-desktop

ARG YOULONG_ROS_DISTRO
ENV ROS_DISTRO=${YOULONG_ROS_DISTRO}
ARG STONEFISH_COMMIT=b21eb8e194c570ff2f61e91aeffb38d73dc25f42
ARG STONEFISH_BUILD_JOBS=1

# stonefish_ros2 is only the ROS wrapper; the Stonefish 1.6 core library is
# an external dependency and is not included in the official ROS image.
RUN set -eux; \
    apt-get update; \
    apt-get install -y --no-install-recommends \
        software-properties-common; \
    add-apt-repository -y universe; \
    apt-get update; \
    apt-get install -y --no-install-recommends \
        build-essential \
        ca-certificates \
        cmake \
        curl \
        git \
        libfreetype6-dev \
        libgl1-mesa-dev \
        libgl1-mesa-dri \
        mesa-utils \
        mesa-vulkan-drivers \
        libglu1-mesa-dev \
        libglm-dev \
        libeigen3-dev \
        libpcl-dev \
        pybind11-dev \
        python3-dev \
        python3-pybind11 \
        libsdl2-dev \
        ffmpeg \
        python3-numpy \
        python3-opencv \
        python3-pil \
        python3-tk \
        python3-venv \
        python3-yaml \
        python3-pip \
        pkg-config; \
    python3 -m pip install --no-cache-dir \
        colcon-common-extensions; \
    git clone --depth 1 https://github.com/patrykcieslak/stonefish.git /opt/stonefish; \
    git -C /opt/stonefish fetch --depth 1 origin "${STONEFISH_COMMIT}"; \
    git -C /opt/stonefish checkout --detach "${STONEFISH_COMMIT}"; \
    cmake -S /opt/stonefish -B /opt/stonefish/build \
        -DCMAKE_BUILD_TYPE=Release \
        -DCMAKE_INSTALL_PREFIX=/usr/local \
        -DBUILD_TESTS=OFF \
        -DEMBED_RESOURCES=OFF; \
    cmake --build /opt/stonefish/build \
        --parallel "${STONEFISH_BUILD_JOBS}"; \
    cmake --install /opt/stonefish/build; \
    ldconfig; \
    rm -rf /opt/stonefish/.git /var/lib/apt/lists/*


# iceoryx2 v0.10 uses Rust 1.89 and the Python binding is built from the
# checked-out submodule at container preparation time.  Keep the toolchain in
# the image, while keeping the source and the generated wheel in /workspace so
# Compose can reuse them across container recreations.
RUN mkdir -p /opt/rust \
    && CARGO_HOME=/opt/rust/cargo RUSTUP_HOME=/opt/rust/rustup \
       sh -c 'curl --proto "=https" --tlsv1.2 -sSf https://sh.rustup.rs | \
              sh -s -- -y --profile minimal --default-toolchain 1.89.0 --no-modify-path' \
    && chmod -R a+rX /opt/rust

ENV RUSTUP_HOME=/opt/rust/rustup \
    PATH=/opt/rust/cargo/bin:${PATH}

# Ubuntu 20.04 ships Pillow 7, while uv_camera requires Pillow >= 9.  The
# same image also needs PySide6 for the uv_record player and visualization tools.
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

# Development image: the repository is bind-mounted at /workspace at runtime.
# The mounted workspaces are built by compose.yaml so their build/install/log
# directories stay in the actual development workspace.

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
        python3-pyqt5.qtwebengine \
    && locale-gen en_US.UTF-8 zh_CN.UTF-8 \
    && fc-cache -f -v \
    && test "$(fc-match -f '%{family}' 'sans-serif:lang=zh-cn' | head -n 1)" = "Noto Sans CJK SC" \
    && rm -rf /var/lib/apt/lists/*

# ROS 2 Foxy rqt_py_common only accepts `pkg/msg/Type` in get_message_class.
# Action feedback topics use `pkg/action/Action_FeedbackMessage`; the generated
# wrapper class exists in the action module, but Foxy's loader rejects the
# valid type string as malformed. Patch the loader in the image so the normal
# rqt Topic Monitor can inspect action feedback topics too.
RUN python3 - <<'PY'
from pathlib import Path

helper = '''\
\ndef _get_action_feedback_class(message_type, logger):
    parts = message_type.split('/')
    if (len(parts) != 3 or parts[1] != 'action' or
            not parts[2].endswith('_FeedbackMessage')):
        return None

    package, _, wrapper_name = parts
    action_name = wrapper_name[:-len('_FeedbackMessage')]
    try:
        action_package = importlib.import_module('%s.action' % package)
        action_class = getattr(action_package, action_name)
        action_module = importlib.import_module(action_class.__module__)
        return getattr(action_module, wrapper_name)
    except (ImportError, AttributeError):
        logger.info('Failed to load action feedback class: {}'.format(message_type))
        return None
'''

needle = '_message_class_cache = {}\n\n\ndef get_message_class(message_type):'
replacement = helper + '\n\n_message_class_cache = {}\n\n\ndef get_message_class(message_type):'
paths = list(Path('/opt/ros').glob('*/lib/python*/site-packages/rqt_py_common/message_helpers.py'))
if not paths:
    raise SystemExit('rqt_py_common/message_helpers.py not found')
for path in paths:
    text = path.read_text()
    if '_get_action_feedback_class' not in text:
        if needle not in text:
            raise SystemExit('unexpected rqt_py_common layout: %s' % path)
        text = text.replace(needle, replacement, 1)
    old = '    class_val = _get_rosidl_class_helper(message_type, MSG_MODE, logger)\n'
    new = '''    class_val = _get_action_feedback_class(message_type, logger)
    if class_val is not None:
        _message_class_cache[message_type] = class_val
        return class_val

''' + old
    if '    class_val = _get_action_feedback_class(message_type, logger)' not in text:
        if old not in text:
            raise SystemExit('get_message_class layout not found: %s' % path)
        text = text.replace(old, new, 1)
    path.write_text(text)
    print('patched', path)
PY

# ROS's rqt is a Qt/X11 application.  Explicitly select the X11 backend and
# add the CJK directory to Qt's font search path so it does not depend on the
# host font configuration mounted into the container.
ENV QT_QPA_PLATFORM=xcb \
    QT_QPA_FONTDIR=/usr/share/fonts/opentype/noto \
    QT_X11_NO_MITSHM=1 \
    SDL_VIDEODRIVER=x11 \
    SDL_VIDEO_X11_FORCE_EGL=0

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
        'case $- in *i*) ;; *) return ;; esac' \
        'source /opt/ros/${ROS_DISTRO}/setup.bash' \
        'if [ -f /workspace/workspace_auv/install/setup.bash ]; then source /workspace/workspace_auv/install/setup.bash; fi' \
        'if [ -f /workspace/workspace_sim/install/setup.bash ]; then source /workspace/workspace_sim/install/setup.bash; fi' \
        'alias uuv_src="source /workspace/workspace_auv/install/setup.bash && source /workspace/workspace_sim/install/setup.bash"' \
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
