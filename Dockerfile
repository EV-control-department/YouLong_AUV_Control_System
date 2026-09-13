FROM osrf/ros:foxy-desktop

ARG HOST_UID=1000
ARG HOST_GID=1000
ARG HOST_USER=dev

# Give the numeric host UID/GID a name inside the container. This keeps bind
# mounted files owned by the host user without producing "I have no name!".
RUN groupadd --gid "${HOST_GID}" "${HOST_USER}" \
    && useradd --uid "${HOST_UID}" --gid "${HOST_GID}" \
        --create-home --shell /bin/bash "${HOST_USER}"

ENV HOME=/home/${HOST_USER}
USER ${HOST_USER}
WORKDIR /workspace
