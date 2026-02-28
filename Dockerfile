# Dockerfile for Mira AUV Firmware
# Based on ROS2 Jazzy with Ubuntu 24.04 LTS
FROM ros:jazzy

# Set environment variables
ENV DEBIAN_FRONTEND=noninteractive
ENV PYTHONUNBUFFERED=1

# Install system dependencies
RUN apt-get update && apt-get install --no-install-recommends -y \
    curl \
    build-essential \
    cmake \
    pkg-config \
    lld \
    ninja-build \
    python3-pip \
    usbutils

# Install uv (Python package manager)
COPY --from=ghcr.io/astral-sh/uv:0.8.18 /uv /uvx /bin/

# Initialize rosdep
RUN rosdep update

# Set working directory
WORKDIR /workspace

# Copy project files so it uses your local fixes!
COPY . /workspace

# Install Python dependencies using uv
RUN uv sync

# Install ROS dependencies
RUN /bin/bash -c "source /opt/ros/jazzy/setup.bash && \
    make install-deps"

RUN rm -rf ./build ./log ./install

# Added apt-get update before install to prevent cache miss errors
RUN apt-get update && apt-get install --no-install-recommends -y \
    usbutils \
    gstreamer1.0-plugins-good \
    libgstreamer1.0-dev \
    libgstreamer-plugins-base1.0-dev \
    libgstreamer-plugins-good1.0-dev \
    libgstreamer-plugins-bad1.0-dev \
    libgstrtspserver-1.0-dev \
    gstreamer1.0-tools \
    gstreamer1.0-plugins-base-apps \
    vim

# Build the workspace
RUN /bin/bash -c "make build"

# Create a script to source the environment
RUN echo '#!/bin/bash\n\
source /opt/ros/jazzy/setup.bash\n\
source install/setup.bash\n\
exec "$@"' > /entrypoint.sh && \
    chmod +x /entrypoint.sh

# Set the entrypoint
ENTRYPOINT ["/entrypoint.sh"]

# Default command
CMD ["bash"]

# Expose common ROS ports
EXPOSE 11311 14550

# Set labels
LABEL maintainer="Mira AUV Team"
LABEL description="Mira AUV Firmware - ROS2 Jazzy based autonomous underwater vehicle control system"
LABEL version="0.1.0"