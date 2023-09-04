# ---
# Boilerplate Development Dockerfile
# This Dockerfile is what you would use during development. A lot of the
# layers are fragmented and carefully ordered to minimize rebuild time of the image
# Ofcourse, this comes at a slight cost of layer size overheads. But during development
# it is better to trade that off with modularity and speed of rebuilding.

# For release, you'd chunk the layers together. See .docker/release.Dockerfile
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

# CUSTOM_SSL_CERTS: If you have custom SSL certificates, store them in .docker/
# These are copied to /usr/local/share/ca-certificates/
ENV INSTALL_CUSTOM_CERT=false
ENV CERT_FILE=""
COPY .docker/*.crt /usr/local/share/ca-certificates/

# CUSTOM_SSL_CERTS: If there was a custom cert file, install it
# and set environment variable INSTALL_CUSTOM_CERT=true, which can be
# used for setting custom SSL certs in other applications
RUN if ls /usr/local/share/ca-certificates/*.crt >/dev/null 2>&1; then \
    echo "INSTALL_CUSTOM_CERT=true" >> /etc/environment; \
    export CERT_FILE=$(ls /usr/local/share/ca-certificates/*.crt | head -n 1); \
    apt-get update; \
    apt-get -y install --no-install-recommends ca-certificates ; \
    update-ca-certificates ; \
    rm -rf /var/lib/apt/lists/*; \
    fi


# APT- Install apt dependencies
# A common list that i use for most projects, includes gl libraries required for GUI apps
RUN apt-get update -y \
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
    # Remove apt cache to not clutter image layers
    && rm -rf /var/lib/apt/lists/* \
    # Add convenience aliases to bashrc
    && echo "alias python=python3" >> /root/.bashrc\
    && echo "alias pip=pip3" >> /root/.bashrc

# ROS
RUN echo "deb http://packages.ros.org/ros/ubuntu focal main" > /etc/apt/sources.list.d/ros1-latest.list \
    && apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 --recv-keys C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654 \
    && apt-get update && apt-get install -y --no-install-recommends \
    ros-${ROS_DISTRO}-ros-core=1.5.0-1* \
    python3-rospy \
    python3-rosdep \
    python3-rosinstall \
    python3-catkin-tools \
    ros-${ROS_DISTRO}-realsense2-camera \
    && rosdep init \
    && rm -rf /var/lib/apt/lists/*

# ROS: Configure Bashrc convenience, if using ROS
RUN echo "WS_DIR=${WS_DIR}" >> ~/.bashrc \
    && echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> ~/.bashrc \
    && echo "[ -f \"\${WS_DIR}/devel/setup.bash\" ] && source \"\${WS_DIR}/devel/setup.bash\"" >> ~/.bashrc \
    && echo "alias source-devel="source "${WS_DIR}/devel/setup.bash""" >> ~/.bashrc

# Global CA Cert config for 3rd part apps, if custom cert file exists
RUN if [ "${INSTALL_CUSTOM_CERT}" = "true" ]; then \
    CERT_PATH="$(python3 -m certifi)" && \
    cp "${CERT_FILE}" "$(openssl version -d | cut -f2 -d \")/certs" && \
    cat "${CERT_FILE}" >> "${CERT_PATH}" && \
    echo "export CERT_PATH=\${CERT_PATH}" >> ~/.bashrc && \
    echo "export SSL_CERT_FILE=\${CERT_PATH}"  >> ~/.bashrc && \
    echo "export REQUESTS_CA_BUNDLE=\${CERT_PATH}"  >> ~/.bashrc; \
    fi



## NOTE: Separating COPY/RUN commands into several layers saves time in rebuilding during development
# During deployment/distribution: To optimize image size, they can be combined into one layer

# Python dependencies from requirements.txt
# It also makes sense to install heavy libraries like torch ahead of development packages
# This saves time for rebuilds, if in-context package is changing
COPY ./requirements.txt ${WS_DIR}/src/requirements.txt
RUN pip install --no-cache-dir torch==1.13.1 --extra-index-url https://download.pytorch.org/whl/cu117 \
    && pip install --no-cache-dir -r ${WS_DIR}/src/requirements.txt \
    && rm ${WS_DIR}/src/requirements.txt

# Python package installation with source code
COPY ./my_python_package1 ${WS_SRC_DIR}/my_python_package1

RUN ${WS_SRC_DIR}/my_python_package1 \
    && git config --global --add safe.directory . \
    && pip install --no-cache-dir -r requirements.txt \
    && pip install --no-cache-dir -e .

# Separate other python packages into their own layers, if they're in development
# This saves time for rebuild, if only one package is changed
COPY ./my_python_package2 ${WS_SRC_DIR}/my_python_package2

RUN ${WS_SRC_DIR}/my_python_package2 \
    && git config --global --add safe.directory . \
    && pip install --no-cache-dir -r requirements.txt \
    && pip install --no-cache-dir -e .


# ROS Packages
# If using sources for external packages, build them first
COPY ext_ros_pkg_source1 ${WS_SRC_DIR}/ext_ros_pkg_source1
COPY ext_ros_pkg_source2 ${WS_SRC_DIR}/ext_ros_pkg_source2

# Install ROS dependencies/packages
RUN apt-get update  \
    && source /opt/ros/${ROS_DISTRO}/setup.bash \
    && cd ${WS_DIR} \
    && rosdep update \
    && rosdep install -y -r -i --rosdistro "${ROS_DISTRO}" --from-paths "${WS_DIR}/src" \
    && catkin build \
    && rm -rf /var/lib/apt/lists/*

# Copy source code
COPY my_ros_package ${WS_SRC_DIR}/my_ros_package

# Build ROS packages
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
# CMD [ "/bin/bash", "-c", "roslaunch my_package something.launch"]
CMD ["/bin/bash"]
