# ---
# Boilerplate Dockerfile
# This Dockerfile is what you would use after development e.g. release. A lot of the
# layers are chunked together to minimize the number of layers and size
# of the image. This is not ideal for development, as it would require
# rebuilding large layers for every small change.

# For development use, see the .docker/development.Dockerfile boilerplate instead.
# ---


# Use arguments for flexible builds
ARG CUDA_VERSION=11.7.1
ARG CUDNN_VERSION=cudnn8
ARG UBUNTU_VERSION=20.04

# Base image
FROM nvidia/cuda:${CUDA_VERSION}-${CUDNN_VERSION}-devel-ubuntu${UBUNTU_VERSION} as base

# ENV- Environment Variables
ENV HOME_DIR=/root/
ENV WS_DIR=${HOME_DIR}/workspaces
ENV WS_SRC_DIR=${WS_DIR}/src
ENV WS_BUILD_DIR=${WS_DIR}/build
ENV WS_INSTALL_DIR=${WS_DIR}/devel
ENV LANG C.UTF-8
ENV LC_ALL C.UTF-8
ENV ROS_DISTRO noetic

# Set SHELL to bash
SHELL ["/bin/bash", "-c"]


# APT- Install apt dependencies
# A common list that i use for most projects, includes gl libraries required for GUI apps
RUN echo "deb http://packages.ros.org/ros/ubuntu focal main" > /etc/apt/sources.list.d/ros1-latest.list \
    && apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 --recv-keys C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654 \
    && apt-get update -y\
    && DEBIAN_FRONTEND=noninteractive \
    apt-get install -q -y --no-install-recommends \
    build-essential \
    curl \
    cmake \
    dirmngr \
    gnupg2 \
    git \
    iputils-ping \
    nano \
    net-tools \
    python3-dev \
    python3-pip \
    python3-wheel \
    tree \
    unzip \
    wget \
    # GUI dependencies
    python3-opengl \
    libegl1-mesa-dev \
    libgl1-mesa-dev \
    libgles2-mesa-dev \
    libglvnd-dev  \
    libglu1-mesa \
    libsm6 \
    libxi6 \
    libxrandr2 \
    libxt6 \
    vulkan-tools \
    qtbase5-dev \
    libqt5core5a \
    libqt5gui5 \
    libqt5widgets5 \
    # ROS
    ros-${ROS_DISTRO}-ros-core=1.5.0-1* \
    python3-rospy \
    python3-rosdep \
    python3-rosinstall \
    python3-catkin-tools \
    ros-${ROS_DISTRO}-realsense2-camera \
    && rosdep init \
    && rm -rf /var/lib/apt/lists/* \
    # bashrc convenience
    && echo "alias python=python3" >> /root/.bashrc\
    && echo "alias pip=pip3" >> /root/.bashrc \
    && echo "WS_DIR=${WS_DIR}" >> ~/.bashrc \
    && echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> ~/.bashrc \
    && echo "[ -f "${WS_DIR}/devel/setup.bash" ] && source "${WS_DIR}/devel/setup.bash"" >> ~/.bashrc \
    && echo "alias source-devel="source ${WS_DIR}/devel/setup.bash"" >> ~/.bashrc


COPY . ${WS_DIR}/src

RUN pip install --no-cache-dir torch==1.13.1 --extra-index-url https://download.pytorch.org/whl/cu117 \
    && pip install --no-cache-dir -r ${WS_DIR}/src/requirements.txt \
    && rm ${WS_DIR}/src/requirements.txt \
    && ${WS_SRC_DIR}/my_python_package1 \
    && git config --global --add safe.directory . \
    && pip install --no-cache-dir -r requirements.txt \
    && pip install --no-cache-dir .\
    && ${WS_SRC_DIR}/my_python_package2 \
    && git config --global --add safe.directory . \
    && pip install --no-cache-dir -r requirements.txt \
    && pip install --no-cache-dir -e .

# Install ROS dependencies/packages
RUN apt-get update  \
    && source /opt/ros/${ROS_DISTRO}/setup.bash \
    && cd ${WS_DIR} \
    && rosdep update \
    && rosdep install -y -r -i --rosdistro "${ROS_DISTRO}" --from-paths "${WS_DIR}/src" \
    && catkin build \
    && rm -rf /var/lib/apt/lists/*

# Entrypoint script
COPY .docker/entrypoint.sh /root/entrypoint.sh
ENTRYPOINT [ "/root/entrypoint.sh" ]

# Drop to bash by default. cli commands override this
# CMD or cli command are passed to entrypoint as arguments
CMD [ "/bin/bash", "-c", "roslaunch my_package something.launch"]
