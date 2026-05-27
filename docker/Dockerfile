# =========================================================================
# Base Image
# =========================================================================
# - https://hub.docker.com/r/hhanoo/ros/tags
FROM hhanoo/ros:humble-desktop

# - Build dependencies
ARG DEBIAN_FRONTEND=noninteractive

# =========================================================================
# Locale and Font
# =========================================================================
RUN apt-get update && apt-get install -y --no-install-recommends \
    locales \
    fonts-noto-color-emoji \
    && rm -rf /var/lib/apt/lists/* \
    && locale-gen en_US en_US.UTF-8 \
    && update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8

ENV LANG=en_US.UTF-8 \
    LANGUAGE=en_US:en \
    LC_ALL=en_US.UTF-8

# =========================================================================
# Essential Packages
# =========================================================================
RUN apt-get update && apt-get install -y --no-install-recommends \
    apt-transport-https \
    apt-utils \
    ca-certificates \
    curl \
    git \
    gnupg2 \
    lsb-release \
    software-properties-common \
    sudo \
    wget \
    && rm -rf /var/lib/apt/lists/*

# =========================================================================
# ROS2
# =========================================================================
# - ROS2 setup
ARG ROS_VERSION=humble
ENV ROS_DISTRO=${ROS_VERSION}

# - ROS2 Vision / Image transport
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-${ROS_DISTRO}-cv-bridge \
    ros-${ROS_DISTRO}-vision-msgs \
    ros-${ROS_DISTRO}-image-transport \
    ros-${ROS_DISTRO}-image-transport-plugins \
    ros-${ROS_DISTRO}-camera-info-manager \
    && rm -rf /var/lib/apt/lists/*

# =========================================================================
# Python
# =========================================================================
RUN apt-get update && apt-get install -y --no-install-recommends \
    python3 \
    python3-pip \
    python3-dev \
    python3-vcstool \
    python3-rosdep \
    && rm -rf /var/lib/apt/lists/*

# - Upgrade pip and install ROS2-compatible setuptools/packaging
RUN python3 -m pip install --no-cache-dir --upgrade \
    pip \
    setuptools==58.2.0 \
    packaging==21.3 \
    wheel

# - Python packages (system-wide)
#   NumPy must be <2.0 for cv_bridge and scipy compatibility
RUN python3 -m pip install --no-cache-dir \
    'numpy>=1.21.0,<2.0' \
    dt-apriltags==3.1.7 \
    opencv-python==4.10.0.84 \
    PyQt5 \
    PyQt5-sip

# =========================================================================
# Development Tools
# =========================================================================
RUN apt-get update && apt-get install -y --no-install-recommends \
    bear \
    clang-format \
    clangd \
    && rm -rf /var/lib/apt/lists/*

# =========================================================================
# C++ Build Dependencies
# =========================================================================
# - libeigen3-dev    : apriltag_pose_estimator
# - libopencv-dev    : apriltag_pose_estimator
# - libapriltag-dev  : apriltag_pose_estimator
# - libmodbus-dev    : tool I/O
# - libusb-1.0-0-dev : USB camera drivers
RUN apt-get update && apt-get install -y --no-install-recommends \
    build-essential \
    cmake \
    pkg-config \
    libeigen3-dev \
    libopencv-dev \
    libapriltag-dev \
    libmodbus-dev \
    libusb-1.0-0 \
    libusb-1.0-0-dev \
    udev \
    && rm -rf /var/lib/apt/lists/*

# =========================================================================
# Runtime Setup
# =========================================================================
# - USB device access groups
RUN groupadd -f plugdev && \
    groupadd -f dialout && \
    usermod -aG plugdev root && \
    usermod -aG dialout root && \
    usermod -aG video root

# =========================================================================
# Workspace Setup
# =========================================================================
# - Workspace
WORKDIR /ros2_ws

# - Commands
COPY ./commands.sh /etc/profile.d/commands.sh
RUN chmod 644 /etc/profile.d/commands.sh && \
    echo 'source /etc/profile.d/commands.sh' >> /etc/bash.bashrc

# - Entrypoint
COPY ./entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh
ENTRYPOINT ["/entrypoint.sh"]

# - Default command
CMD ["bash"]
