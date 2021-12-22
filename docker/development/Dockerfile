# syntax=docker/dockerfile:1
FROM ubuntu:20.04

# Setup timezone
RUN echo 'Etc/UTC' > /etc/timezone \
    && ln -s /usr/share/zoneinfo/Etc/UTC /etc/localtime \
    && apt-get update \
    && apt-get install -q -y --no-install-recommends tzdata \
    && rm -rf /var/lib/apt/lists/*

# Create the image file structure
RUN mkdir /home/catkin_ws \
    && mkdir /home/catkin_ws/src

# Setup the environments
ENV LANG=C.UTF-8
ENV LC_ALL=C.UTF-8
ENV ROS_DISTRO=noetic
ENV HOME=/home/
ENV ROS_WS=/home/catkin_ws/
WORKDIR ${HOME}

# Install some helpful utilities
RUN apt-get update && apt-get install -y --no-install-recommends \
    iputils-ping \
    net-tools \
    nano \
    vim \
    git \
    clang \
    g++ \
    rsync \
    zip \
    make \
    cmake \
    curl \
    wget \
    libopencv-dev \
    python3-opencv \
    build-essential \
    lsb-release \
    ca-certificates \
    dirmngr \
    gnupg2 \
    python3 \
    python3-pip \
    netbase \
    htop \
    nmap \
    && rm -rf /var/lib/apt/lists/*

# Install Python packages
RUN pip3 install \
    numpy \
    pandas \
    scipy \
    matplotlib \
    pyserial \
    pymavlink

# Add the ROS repo to the sources list
RUN sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'

# Add the ROS key
RUN curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | apt-key add -

# Install ROS Noetic
RUN apt-get update && apt-get install -y --no-install-recommends \
    python3-rosdep \
    python3-rosinstall \
    python3-vcstools \
    ros-noetic-ros-base \
    && rosdep init \
    && rosdep update \
    && rm -rf /var/lib/apt/lists/*

# Install MAVROS and other ROS dependencies
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-noetic-mavros \
    ros-noetic-mavros-extras \
    ros-noetic-tf \
    ros-noetic-diagnostic-updater \
    && sudo /opt/ros/${ROS_DISTRO}/lib/mavros/install_geographiclib_datasets.sh \
    && rm -rf /var/lib/apt/lists/*

# Copy the swarm nodes to the image
COPY swarm-nodes /home/catkin_ws/src/

# Build the ROS workspace
RUN . /opt/ros/${ROS_DISTRO}/setup.sh \
    && cd ${ROS_WS} \
    && catkin_make

# Overlay the ROS environment on top of the existing environment
COPY docker/development/ros_entrypoint.sh /
RUN ["chmod", "+x", "/ros_entrypoint.sh"]
ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["bash"]