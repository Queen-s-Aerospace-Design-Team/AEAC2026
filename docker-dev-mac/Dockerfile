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
# Workspace and ArduPilot
# --------------------------

WORKDIR $HOME_DIR

# Clone ArduPilot
RUN git clone --depth 1 https://github.com/ArduPilot/ardupilot.git

# Install ArduPilot prereqs
WORKDIR $HOME_DIR/ardupilot
RUN Tools/environment_install/install-prereqs-ubuntu.sh -y \
 && pip3 install --no-cache-dir -U pip future pymavlink MAVProxy

# Pre-build SITL (speeds up first run)
RUN ./waf configure --board sitl && ./waf copter

# Convenience: autotest tools on PATH
ENV PATH="$HOME_DIR/ardupilot/Tools/autotest:${PATH}"

# --------------------------
# User, dotfiles, entry
# --------------------------

# Add new user 'qadt'
RUN useradd -ms /bin/bash qadt \
 && echo "qadt:aero" | chpasswd \
 && usermod -aG sudo qadt

# (Optional) bring your shell/dev dotfiles if present in build context
# Comment out any that you don't have.
# COPY .bashrc .profile .tmux.conf $HOME_DIR/

# Run script that launches SITL -> QGC on the Mac host (14550)
# Use the run_sitl.sh from earlier; keep it in your build context.
COPY run_sitl.sh /usr/local/bin/run_sitl.sh
RUN chmod +x /usr/local/bin/run_sitl.sh \
 && chown -R qadt:qadt $HOME_DIR

USER qadt
WORKDIR $HOME_DIR

# Mac-safe: use host.docker.internal (Docker Desktop for Mac)
ENTRYPOINT ["/usr/local/bin/run_sitl.sh"]