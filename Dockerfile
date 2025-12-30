FROM ubuntu:24.04

ENV DEBIAN_FRONTEND=noninteractive
SHELL ["/bin/bash", "-c"]

# ---- Base tools ----
RUN apt-get update && apt-get install -y --no-install-recommends \
    ca-certificates \
    curl \
    gnupg \
    lsb-release \
    locales \
    software-properties-common \
    build-essential \
    git \
    python3-pip \
 && rm -rf /var/lib/apt/lists/*

# ---- Locale ----
RUN locale-gen en_US.UTF-8 && update-locale LANG=en_US.UTF-8
ENV LANG=en_US.UTF-8
ENV LC_ALL=en_US.UTF-8

# Universe repo
RUN add-apt-repository universe

# ---- ROS 2 repository ----
RUN mkdir -p /usr/share/keyrings \
 && curl -fsSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
    -o /usr/share/keyrings/ros-archive-keyring.gpg \
 && echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
    http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" \
    > /etc/apt/sources.list.d/ros2.list

# ---- Gazebo OSRF repo ----
# ---- Gazebo OSRF repo (try HTTP if HTTPS blocked) ----
RUN curl -fsSL http://packages.osrfoundation.org/gazebo.gpg \
    -o /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg \
 && echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] \
    http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" \
    > /etc/apt/sources.list.d/gazebo-stable.list


# ---- Install ROS 2 Kilted + requested packages + Gazebo Harmonic ----
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-kilted-desktop \
    python3-colcon-common-extensions \
    python3-rosdep \
    ros-kilted-ros2-controllers \
    ros-kilted-gz-ros2-control \
    ros-kilted-ros-gz \
    ros-kilted-ros-gz-bridge \
    ros-kilted-joint-state-publisher \
    ros-kilted-robot-state-publisher \
    ros-kilted-xacro \
    ros-kilted-joy \
    gz-harmonic \
 && rm -rf /var/lib/apt/lists/*

# ---- rosdep ----
RUN rosdep init || true && rosdep update

# Auto-source ROS
RUN echo "source /opt/ros/kilted/setup.bash" >> /root/.bashrc

WORKDIR /ros_ws
CMD ["bash"]

