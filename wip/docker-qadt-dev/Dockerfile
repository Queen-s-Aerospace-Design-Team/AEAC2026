# --------------------------
# Dev: sources + workspace
# --------------------------

FROM qadt-base:latest
ARG ROS_DISTRO=humble
ENV HOME_DIR=/home/qadt
SHELL ["/bin/bash","-c"]

USER root

# System deps
RUN apt-get update && apt-get install -y --no-install-recommends \
    gdb \
    lcov \
    python3 \
    python3-dev \
    python3-pip \
    python3-setuptools \
    python3-wheel \
    python3-rosdep \
    python3-vcstool \
    libssl-dev \
    libxml2-dev \
    libxml2-utils \
    tmux \
	# For XRCE-DDS Gen
	default-jre \
    # Clear cache
    && rm -rf /var/lib/apt/lists/*

WORKDIR ${HOME_DIR}

# Copy required dotfiles to home directory
COPY .bashrc .profile .tmux.conf ${HOME_DIR}

# Install ArduPilot ROS2 repos via vcstool (versioned)
# Keep repos file in the context for reproducibility
RUN mkdir -p ${HOME_DIR}/ArduPilot/src \
 	&& vcs import --recursive --input  https://raw.githubusercontent.com/ArduPilot/ardupilot/master/Tools/ros2/ros2.repos ${HOME_DIR}/ArduPilot/src


# Clone uisng github --recursive --depth 1, maybe specific branch for ubuntu 22.04 and for ROS2 Humble?
# Upload the built firmware to the flight controller hardware.
# -----------------
# RUN git clone --recurse-submodules --depth 1 https://github.com/ArduPilot/ardupilot.git \
#       && ./Tools/environment_install/install-prereqs-ubuntu.sh -y


# Gazebo Harmonic install LTS sudo apt-get install curl lsb-release gnupg
# -----------------
# RUN sudo apt-get install curl lsb-release gnupg

# Resolve ROS deps (must preform as root user and tell rosdep specifically with --as-root). Use --rosdistro to avoid sourcing at build.
RUN apt-get update \
	&& rosdep update \
 	&& rosdep install --from-paths ${HOME_DIR}/ArduPilot/src --ignore-src -r -y --rosdistro ${ROS_DISTRO} --as-root apt:true

# Micro XRCE-DDS Generator for ArduPilot
RUN git clone --depth 1 https://github.com/ardupilot/Micro-XRCE-DDS-Gen.git \
 	&& cd Micro-XRCE-DDS-Gen && ./gradlew --no-daemon assemble

# Have user 'qadt' own everything in home
RUN chown -R qadt:qadt ${HOME_DIR}

# Switch to user and launch entrypoint
USER qadt
WORKDIR ${HOME_DIR}
ENTRYPOINT ["/usr/local/bin/entrypoint.sh"]
CMD ["bash"]
