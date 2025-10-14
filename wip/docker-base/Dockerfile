# --------------------------
# Base: OS + ROS
# --------------------------

ARG ROS_DISTRO=humble
FROM ros:${ROS_DISTRO}

ARG DEBIAN_FRONTEND=noninteractive
ARG USER=qadt
ARG UID=1000
ARG GID=1000

# Set env variables
ENV TZ=UTC \
    LANG=C.UTF-8 \
    LC_ALL=C.UTF-8 \
    ROS_DISTRO=${ROS_DISTRO} \
    HOME_DIR=/home/${USER} \
    DOCKER_BUILDKIT=1

SHELL ["/bin/bash","-c"]

# System deps
RUN apt-get update && apt-get install -y --no-install-recommends \
    wget \
    curl \
    build-essential \
    cmake \
    ninja-build \
    ca-certificates \
    gnupg \
    build-essential \
    ccache \
    git \
    shellcheck \
    cppcheck \
    astyle \
    file \
    rsync \
    unzip \
    zip \
    vim \
    nano \
    # Clear cache
    && rm -rf /var/lib/apt/lists/*

# rosdep setup. Run once
RUN rosdep init || true

# Create non-root user and add useful groups
RUN groupadd -g ${GID} ${USER} \
    && useradd -m -u ${UID} -g ${GID} -s /bin/bash ${USER} \
    && usermod -aG sudo,dialout,video ${USER} \
    && echo "${USER}:aero" | chpasswd

# Entrypoint
COPY entrypoint.sh /usr/local/bin/entrypoint.sh
RUN chmod +x /usr/local/bin/entrypoint.sh && chown ${USER}:${USER} /usr/local/bin/entrypoint.sh

# Switch to user and launch entry point
USER ${USER}
WORKDIR ${HOME_DIR}
ENTRYPOINT ["/usr/local/bin/entrypoint.sh"]
CMD ["bash"]
