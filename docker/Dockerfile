# ROS 2 Humble Desktop base -------------------------------------------------
# https://hub.docker.com/_/ros/tags
FROM hhanoo/ros:humble-desktop

# Locale and Font Settings --------------------------------------------------
RUN apt-get update && apt-get install -y --no-install-recommends \
    locales \
    fonts-noto-color-emoji \
    && rm -rf /var/lib/apt/lists/* \
    && locale-gen en_US.UTF-8

ENV LANG=en_US.UTF-8 \
    LANGUAGE=en_US:en \
    LC_ALL=en_US.UTF-8

# Build Dependencies --------------------------------------------------------
RUN apt-get update && apt-get install -y --no-install-recommends \
    build-essential \
    cmake \
    git \
    && rm -rf /var/lib/apt/lists/*

# Tool Dependencies ---------------------------------------------------------
RUN apt-get update && apt-get install -y --no-install-recommends \
    libmodbus-dev \
    libusb-1.0-0 \
    libusb-1.0-0-dev \
    udev \
    && rm -rf /var/lib/apt/lists/*

# Development Tools ---------------------------------------------------------
RUN apt-get update && apt-get install -y --no-install-recommends \
    bear \
    clang-format \
    clangd \
    && rm -rf /var/lib/apt/lists/*

# Intel RealSense SDK -------------------------------------------------------
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-humble-realsense2-* \
    && rm -rf /var/lib/apt/lists/*

# Vision Dependencies -------------------------------------------------------
RUN apt-get update && apt-get install -y --no-install-recommends \
    libeigen3-dev \
    libopencv-dev \
    libapriltag-dev \
    && rm -rf /var/lib/apt/lists/*

# Python base ---------------------------------------------------------------
RUN apt-get update && apt-get install -y --no-install-recommends \
    python3 \
    python3-pip \
    python3-dev \
    python3-vcstool \
    python3-rosdep \
    ca-certificates \
    curl \
    && rm -rf /var/lib/apt/lists/*

# Upgrade pip and install ROS2-compatible setuptools/packaging --------------
RUN python3 -m pip install --no-cache-dir --upgrade \
    pip \
    setuptools==58.2.0 \
    packaging==21.3 \
    wheel

# Python Dependencies (system-wide) -----------------------------------------
# Note: NumPy must be <2.0 for cv_bridge and scipy compatibility
RUN python3 -m pip install --no-cache-dir \
    'numpy>=1.21.0,<2.0' \
    dt-apriltags==3.1.7 \
    opencv-python==4.10.0.84 \
    pyrealsense2==2.56.5.9235 \
    PyQt5 \
    PyQt5-sip

# USB device access groups --------------------------------------------------
RUN groupadd -f plugdev && \
    groupadd -f dialout && \
    usermod -aG plugdev root && \
    usermod -aG dialout root && \
    usermod -aG video root

# Setup workspace -----------------------------------------------------------
WORKDIR /ros2_ws

# ROS auto source (no venv) -------------------------------------------------
RUN echo "source /opt/ros/humble/setup.bash" >> /root/.bashrc && \
    echo "if [ -f /ros2_ws/install/setup.bash ]; then source /ros2_ws/install/setup.bash; fi" >> /root/.bashrc

# Default command -----------------------------------------------------------
CMD ["bash"]