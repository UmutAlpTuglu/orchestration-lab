# test locally inside ws folder
# docker build -t pointcloud_detection -f pointcloud.Dockerfile .
# docker run --rm -it pointcloud_detection /bin/bash
#inside container: ros2 pkg list | grep pointcloud_detection

FROM ros:iron-ros-base

ARG USERNAME=vscode
ARG USER_UID=1000
ARG USER_GID=1000

# Create the user
RUN groupadd --gid $USER_GID $USERNAME \
    && useradd -s /bin/bash --uid $USER_UID --gid $USER_GID -m $USERNAME \
    && apt-get update \
    && apt-get install -y sudo \
    && echo $USERNAME ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/$USERNAME \
    && chmod 0440 /etc/sudoers.d/$USERNAME

# Install development tools and Python dependencies
RUN apt-get update && apt-get upgrade -y && \
    apt-get install -y \
    python3-pip \
    python3-pytest-cov \
    ros-iron-ament-* \
    ros-iron-ros-testing \
    python3-colcon-common-extensions \
    git \
    && rm -rf /var/lib/apt/lists/*

# Create and set permissions for workspace
WORKDIR /home/ws
RUN mkdir -p /home/ws \
    && chown -R $USERNAME:$USERNAME /home/ws

# Copy package files with correct ownership
COPY --chown=$USERNAME:$USERNAME . /home/ws/src/pointcloud_detection/

# Switch to non-root user
USER $USERNAME

# Create workspace directories with correct permissions
RUN mkdir -p /home/ws/src \
    && mkdir -p /home/ws/build \
    && mkdir -p /home/ws/install \
    && mkdir -p /home/ws/log

# Build the package
RUN /bin/bash -c 'source /opt/ros/iron/setup.bash && colcon build --symlink-install'

# Set up ROS environment
RUN echo "source /opt/ros/iron/setup.bash" >> /home/$USERNAME/.bashrc
RUN echo "[ -f /home/ws/install/setup.bash ] && source /home/ws/install/setup.bash" >> /home/$USERNAME/.bashrc

ENV SHELL=/bin/bash

# Command to run your node
CMD ["ros2", "run", "pointcloud_detection", "pointcloud_detection"]