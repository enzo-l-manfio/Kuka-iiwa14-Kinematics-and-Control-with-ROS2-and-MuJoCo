FROM osrf/ros:jazzy-desktop

ARG USERNAME=rosuser
ARG USER_UID=1000
ARG USER_GID=$USER_UID

# Remove existing 'ubuntu' user (UID/GID 1000) if present, then create rosuser
RUN if id "ubuntu" &>/dev/null; then userdel -r ubuntu; fi \
    && groupadd --gid $USER_GID $USERNAME \
    && useradd --uid $USER_UID --gid $USER_GID -m $USERNAME \
    && apt-get update && apt-get install -y sudo \
    && echo $USERNAME ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/$USERNAME \
    && chmod 0440 /etc/sudoers.d/$USERNAME

# Install system dependencies, build tools, and pip
RUN apt-get update && apt-get install -y \
    python3-colcon-common-extensions \
    python3-rosdep \
    python3-pip \
    ros-jazzy-xacro


# Install Python packages
RUN pip3 install --no-cache-dir --break-system-packages roboticstoolbox-python \
    spatialmath-python[ros-jazzy]


#Install C++ libraries
RUN apt-get install -y \
    libeigen3-dev \
    ros-jazzy-pinocchio \
    && rm -rf /var/lib/apt/lists/*

#MuJoCo installation and environment variables configuration
ARG MUJOCO_VERSION=3.2.6
RUN mkdir -p /opt/mujoco && \
    curl -sSL https://github.com/google-deepmind/mujoco/releases/download/${MUJOCO_VERSION}/mujoco-${MUJOCO_VERSION}-linux-x86_64.tar.gz \
    | tar -xzf - -C /opt/mujoco

RUN cat <<EOF >> /home/$USERNAME/.bashrc
	export MUJOCO_PY_MUJOCO_PATH=/opt/mujoco/mujoco-${MUJOCO_VERSION}
	export LD_LIBRARY_PATH=\$LD_LIBRARY_PATH:/opt/mujoco/mujoco-${MUJOCO_VERSION}/bin
	export LD_LIBRARY_PATH=/opt/mujoco/mujoco-${MUJOCO_VERSION}/lib:\$LD_LIBRARY_PATH
EOF

RUN apt-get update && apt-get install -y libglfw3-dev

# Automatically source ROS 2 in bash sessions
RUN echo "source /opt/ros/$ROS_DISTRO/setup.bash" >> /home/${USERNAME}/.bashrc


ENV TERM=xterm-256color

#Adding personal terminal preferences
COPY docker/bashrc_preferences.sh /usr/local/bin/
RUN touch /home/${USERNAME}/.bashrc && if [ -f /usr/local/bin/bashrc_preferences.sh ]; then \
        chmod +x /usr/local/bin/bashrc_preferences.sh; \
        cat /usr/local/bin/bashrc_preferences.sh   >> /home/${USERNAME}/.bashrc; \
        fi


USER $USERNAME
WORKDIR /home/$USERNAME/kuka_ws
