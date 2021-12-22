# syntax=docker/dockerfile:1
FROM arm64v8/ros:noetic-robot

# Create the image file structure
RUN mkdir /home/catkin_ws \
    && mkdir /home/catkin_ws/src

# Setup the environments
ENV LANG=C.UTF-8
ENV LC_ALL=C.UTF-8
ENV ROS_DISTRO=noetic
ENV HOME=/home/
ENV ROS_WS=/home/catkin_ws/
WORKDIR ${ROS_WS}

# Install utilities/dependencies
RUN apt-get update && apt-get install -y --no-install-recommends \
    iputils-ping \
    net-tools \
    netbase \
    python3 \
    python3-pip \
    git \
    nano \
    vim \
    htop \
    && rm -rf /var/lib/apt/lists/*

# Install Python packages
RUN pip3 install \
    pyserial \
    pymavlink

# Install MAVROS
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-noetic-mavros \
    ros-noetic-mavros-extras \
    && rm -rf /var/lib/apt/lists/*

# Now install weird MAVROS geoids
RUN sudo /opt/ros/${ROS_DISTRO}/lib/mavros/install_geographiclib_datasets.sh

# Copy the swarm nodes to the image
COPY swarm-nodes /home/catkin_ws/src/

# Build the ROS workspace
RUN . /opt/ros/${ROS_DISTRO}/setup.sh \
    && cd ${ROS_WS} \
    && catkin_make

# Overlay the ROS environment on top of the existing environment
COPY docker/drone/ros_entrypoint.sh /
RUN ["chmod", "+x", "/ros_entrypoint.sh"]
ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["bash"]