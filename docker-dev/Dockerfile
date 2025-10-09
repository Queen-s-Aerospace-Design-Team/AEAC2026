# --------------------------
# Base image & environment
# --------------------------

FROM ros:humble

ENV ROS_DISTRO=humble \
	DEBIAN_FRONTEND=noninteractive  \
	DOCKER_BUILDKIT=1 \
	HOME_DIR=/home/qadt

SHELL ["/bin/bash", "-c"] 

# --------------------------
# System Dependencies
# --------------------------

# Update and install necessary dependencies
RUN apt-get update \
    && apt-get install -y --no-install-recommends \
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
    # Put extra apt packages here for example:
    # ros-humble-rmw-fastrtps-cpp \
    && rm -rf /var/lib/apt/lists/*

# --------------------------
# Workspace and Development Dependencies
# --------------------------

# Create your workspace here
WORKDIR $HOME_DIR

# Clone MicroDDS. Adapted from https://docs.px4.io/main/en/ros2/user_guide
# NOTE: Used to be v2.4.2 but updating to v2.4.3 as v2.4.2 had issues with docker build.
# Build MicroDDS using cmake via the ninja-build generator. '-j $(nproc)' will use all processors to speed up build times.
RUN git clone --branch v2.4.3 https://github.com/eProsima/Micro-XRCE-DDS-Agent.git --depth 1 \
	&& cmake -S Micro-XRCE-DDS-Agent -B Micro-XRCE-DDS-Agent/build -G "Ninja" \
    && cmake --build Micro-XRCE-DDS-Agent/build -j $(nproc) \
    && cmake --install Micro-XRCE-DDS-Agent/build \
    && ldconfig \
	&& rm -rf Micro-XRCE-DDS-Agent/build

# Effectively copied 'px4_ubuntu.sh' from "bash ./Tools/setup/ubuntu.sh" above
# Python dependencies in requirements.txt
COPY px4_ubuntu.sh \
	requirements.txt \
	./

RUN bash px4_ubuntu.sh \
	&& pip3 install --no-cache-dir -r requirements.txt

# Run the setup and build for PX4
RUN git clone --single-branch --branch v1.16.0 https://github.com/PX4/PX4-Autopilot.git --recursive --depth 1 \
	&& make -C PX4-Autopilot

# Download QGroundControl
# RUN apt-get update && apt-get install libfuse2 -y --no-install-recommends \
# 	&& ARCH=$(arch) \
# 	&& curl https://d176tv9ibo4jno.cloudfront.net/latest/QGroundControl-$ARCH.AppImage -O \
# 	&& chmod +x QGroundControl-$ARCH.AppImage 

# Install ArduPilot and MicroXRCEDDSGen
# WORKDIR $HOME_DIR/ArduPilot

# RUN mkdir src \
# 	&& pip3 install --no-cache-dir vcstools \
# 	&& vcs import --recursive --input  https://raw.githubusercontent.com/ArduPilot/ardupilot/master/Tools/ros2/ros2.repos src \
# 	&& rosdep update \
# 	&& source /opt/ros/humble/setup.sh \
# 	&& rosdep install --from-paths src --ignore-src -r -y

# WORKDIR $HOME_DIR

# RUN apt-get install default-jre -y --no-install-recommends \
# 	&& git clone https://github.com/ardupilot/Micro-XRCE-DDS-Gen.git --recursive --depth 1 \
# 	&& cd Micro-XRCE-DDS-Gen && ./gradlew assemble

# Copy necessary files from the dockerfile's directory into the container
COPY .bashrc \
	.profile \
	.tmux.conf \
	entrypoint.sh \
	./

# --------------------------
# Entrypoint and User
# --------------------------

# Add new user 'qadt'
RUN useradd -ms /bin/bash qadt && \
    echo "qadt:aero" | chpasswd && \
    usermod -aG sudo qadt 
	# Uncomment if you want sudo access without a password
	# && echo "qadt ALL=(ALL) NOPASSWD:ALL" > /etc/sudoers.d/qadt \
    # && chmod 0440 /etc/sudoers.d/qadt

# Make user 'qadt' own all files in home directory
RUN chown -R qadt:qadt $HOME_DIR \
	&& chmod +x $HOME_DIR/entrypoint.sh

# Switch to user and run entry point
USER qadt
ENTRYPOINT [ "/home/qadt/entrypoint.sh" ]
CMD [ "bash" ]
