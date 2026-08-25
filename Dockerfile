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
    ros-jazzy-xacro \
    && rm -rf /var/lib/apt/lists/*


# Install Python packages
RUN pip3 install --no-cache-dir --break-system-packages roboticstoolbox-python \
    pip3 install --no-cache-dir --break-system-packages spatialmath-python[ros-jazzy]


#Install C++ libraries
RUN sudo apt install libeigen3-dev ros-jazzy-pinocchio

#MuJoCo installation and environment variables configuration
ARG MUJOCO_VERSION=3.2.6
RUN mkdir -p /opt/mujoco && \
    curl -sSL https://github.com/google-deepmind/mujoco/releases/download/${MUJOCO_VERSION}/mujoco-${MUJOCO_VERSION}-linux-x86_64.tar.gz \
    | tar -xzf - -C /opt/mujoco

RUN cat <<EOF >> /home/$USERNAME/.bashrc
	export MUJOCO_PY_MUJOCO_PATH=/opt/mujoco/${MUJOCO_VERSION}
	export LD_LIBRARY_PATH=\$LD_LIBRARY_PATH:/opt/mujoco/${MUJOCO_VERSION}/bin
	export LD_LIBRARY_PATH=/opt/mujoco/${MUJOCO_VERSION}/lib:\$LD_LIBRARY_PATH
EOF

RUN apt-get update && apt-get install -y libglfw3-dev

# Automatically source ROS 2 in bash sessions
RUN echo "source /opt/ros/$ROS_DISTRO/setup.bash" >> /home/$USERNAME/.bashrc

# Quero corzinha
ENV TERM=xterm-256color

# Personal terminal preferences, feel free to delete
RUN cat <<'EOF' >> /home/$USERNAME/.bashrc
    force_color_prompt=yes
    if [ -n "$force_color_prompt" ]; then
        if [ -x /usr/bin/tput ] && tput setaf 1 >&/dev/null; then
        # We have color support; assume it's compliant with Ecma-48
        # (ISO/IEC-6429). (Lack of such support is extremely rare, and such
        # a case would tend to support setf rather than setaf.)
        color_prompt=yes
        else
        color_prompt=
        fi
    fi

    parse_git_branch() {
        git branch 2> /dev/null | sed -e '/^[^*]/d' -e 's/* \(.*\)/ (\1)/'
    }

    if [ "$color_prompt" = yes ]; then
        PS1='${debian_chroot:+($debian_chroot)}\[\033[01;34m\][\t] \[\033[01;35m\]\[\033[01;32m\]\u@\h\[\033[00m\]:\[\033[01;35m\]$(parse_git_branch) \[\033[01;31m\]\w\[\033[00m\]\n\$ '
    else
        PS1='${debian_chroot:+($debian_chroot)}[\t] \u@\h:$(parse_git_branch)\w\n\$ '
    fi
    unset color_prompt force_color_prompt

    # If this is an xterm set the title to user@host:dir
    case "$TERM" in
    xterm*|rxvt*)
        PS1="\[\e]0;${debian_chroot:+($debian_chroot)}[\t] \u@\h: $(parse_git_branch)\w\a\]$PS1"
        ;;
    *)
        ;;
    esac

    # enable color support of ls and also add handy aliases
    if [ -x /usr/bin/dircolors ]; then
        test -r ~/.dircolors && eval "$(dircolors -b ~/.dircolors)" || eval "$(dircolors -b)"
        alias ls='ls --color=auto'
        #alias dir='dir --color=auto'
        #alias vdir='vdir --color=auto'

        alias grep='grep --color=auto'
        alias fgrep='fgrep --color=auto'
        alias egrep='egrep --color=auto'
    fi

    # colored GCC warnings and errors
    export GCC_COLORS='error=01;31:warning=01;35:note=01;36:caret=01;32:locus=01:quote=01'

    # some more ls aliases
    alias ll='ls -alF'
    alias la='ls -A'
    alias l='ls -CF'
    alias ..='cd ..'
    alias bsource='source ~/.bashrc'
    
    #git
    alias gi='git init'
    alias gst='git status'
    alias ga='git add'
    alias gr='git restore'
    alias gco='git commit'
    alias gps='git push'
    alias gpl='git pull'
    alias gbr='git branch'
    alias gch='git checkout'
    alias gchn='git checkout -b'

    #ROS
    alias rosource='source install/local_setup.bash'
    alias cbuild='colcon build'
    alias cbuild_so='colcon build --packages-select'
EOF


USER $USERNAME
WORKDIR /home/$USERNAME/kuka_ws
