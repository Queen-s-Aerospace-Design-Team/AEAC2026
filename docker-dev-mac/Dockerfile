# --------------------------
# Base image & environment
# --------------------------

FROM ros:humble

ENV ROS_DISTRO=humble \
    DEBIAN_FRONTEND=noninteractive \
    DOCKER_BUILDKIT=1 \
    HOME_DIR=/home/qadt \
    VEHICLE=ArduCopter \
    FRAME=quad \
    SIM_SPEEDUP=1 \
    OUT_TARGET="udp:host.docker.internal:14550"

SHELL ["/bin/bash", "-c"]

# --------------------------
# System Dependencies
# --------------------------

RUN apt-get update \
 && apt-get install -y --no-install-recommends \
    sudo \
    tzdata \
    ca-certificates \
    wget \
    curl \
    astyle \
    build-essential \
    ccache \
    cmake \
    cppcheck \
    file \
    g++ \
    gcc \
    gdb \
    git \
    lcov \
    libssl-dev \
    libxml2-dev \
    libxml2-utils \
    libtool \
    pkg-config \
    make \
    ninja-build \
    python3 \
    python3-dev \
    python3-pip \
    python3-setuptools \
    python3-wheel \
    rsync \
    shellcheck \
    unzip \
    zip \
    vim \
    tmux \
    # Mabye add ROS 2 extras later
 && rm -rf /var/lib/apt/lists/*

# --------------------------
# Create user first (sudo NOPASSWD), set envs
# --------------------------
RUN useradd -ms /bin/bash qadt \
 && echo "qadt ALL=(ALL) NOPASSWD:ALL" > /etc/sudoers.d/qadt \
 && adduser qadt dialout

USER qadt
# Ensure non-interactive shells have a username/home
ENV USER=qadt \
    LOGNAME=qadt \
    HOME=/home/qadt \
    PATH="/home/qadt/.local/bin:${PATH}"

WORKDIR /home/qadt

# --------------------------
# ArduPilot
# --------------------------
RUN git clone --depth 1 https://github.com/ArduPilot/ardupilot.git
WORKDIR /home/qadt/ardupilot

# This script expects a non-root $USER and uses sudo internally
RUN Tools/environment_install/install-prereqs-ubuntu.sh -y \
 && pip3 install --user --no-cache-dir -U pip future pymavlink MAVProxy

# Build SITL
RUN ./waf configure --board sitl && ./waf copter


# --------------------------
# User, dotfiles, entry
# --------------------------

# Switch to root for system paths
USER root

# (Optional) bring your shell/dev dotfiles; set ownership at copy-time
# COPY --chown=qadt:qadt .bashrc .profile .tmux.conf /home/qadt/

# Install the entry script into /usr/local/bin as root
COPY --chown=root:root run_sitl.sh /usr/local/bin/run_sitl.sh
RUN chmod 0755 /usr/local/bin/run_sitl.sh

# If you copied anything into /home/qadt as root, ensure ownership (usually not needed here)
# RUN chown -R qadt:qadt /home/qadt

# Drop privileges for runtime
USER qadt
WORKDIR /home/qadt

ENTRYPOINT ["/usr/local/bin/run_sitl.sh"]
