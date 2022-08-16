# syntax=docker/dockerfile:1
FROM arm64v8/ros:noetic-robot

# Create the image file structure
RUN mkdir /home/catkin_ws \
    && mkdir /home/catkin_ws/src \
    && mkdir /home/rosbagsls

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
    python3-lxml \
    python3-catkin-tools \
    python3-rosinstall-generator \
    geographiclib-tools \
    libgeographic-dev \
    && geographiclib-get-geoids egm96-5 \
    && geographiclib-get-gravity egm96 \
    && geographiclib-get-magnetic emm2015 \
    && rm -rf /var/lib/apt/lists/*


# Install Python packages
RUN pip3 install \
    pyserial \
    pymavlink

# Install ROS packages
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-noetic-control-toolbox \
    ros-noetic-angles

# Build the ROS workspace
RUN . /opt/ros/${ROS_DISTRO}/setup.sh \
    && cd ${ROS_WS} \
    && rosdep update \
    && catkin init \
    && wstool init src

# Install MAVROS
# Note that we perform this step as an independent layer to prevent
# requiring internet to rebuild swarm nodes
COPY swarm-nodes/mavros /home/catkin_ws/src/mavros


# Install MAVLink and configure workspace
RUN rosinstall_generator --rosdistro kinetic mavlink | tee /tmp/mavros.rosinstall \
    && wstool merge -t src /tmp/mavros.rosinstall \
    && wstool update -t src -j4 \
    && rosdep install --from-paths src --ignore-src -y


# We perform a build here to avoid triggering the ungodly long
# mavros builds when code changes are made in the field
RUN . /opt/ros/${ROS_DISTRO}/setup.sh \ 
    && catkin build


# Copy the configuration folder
COPY swarm-nodes/config /home/catkin_ws/src/config


# Copy the swarm ROS nodes into the image
COPY swarm-nodes/custom_libs /home/catkin_ws/src/custom_libs
# COPY swarm-nodes/estimator /home/catkin_ws/src/estimator
COPY swarm-nodes/pre_hrl /home/catkin_ws/src/pre_hrl
COPY swarm-nodes/hrl_lqi /home/catkin_ws/src/hrl_lqi
COPY swarm-nodes/post_hrl /home/catkin_ws/src/post_hrl
COPY swarm-nodes/swarm_msgs /home/catkin_ws/src/swarm_msgs
COPY swarm-nodes/online_hrl.launch  /home/catkin_ws/src/online_hrl.launch

# Rosbag management
COPY docker/drone/bag_manager.sh /home/
RUN ["chmod","+x","/home/bag_manager.sh"]


# Overlay the ROS environment on top of the existing environment
COPY docker/drone/ros_entrypoint.sh /
RUN ["chmod", "+x", "/ros_entrypoint.sh"]
ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["bash"]






## Install MAVROS
#RUN apt-get update && apt-get install -y --no-install-recommends \
#    ros-noetic-mavros \
#    ros-noetic-mavros-extras \
#    && rm -rf /var/lib/apt/lists/*

## Now install weird MAVROS geoids
#RUN sudo /opt/ros/${ROS_DISTRO}/lib/mavros/install_geographiclib_datasets.sh

## Copy the swarm nodes to the image
#COPY swarm-nodes /home/catkin_ws/src/

## Build the ROS workspace
#RUN . /opt/ros/${ROS_DISTRO}/setup.sh \
#    && cd ${ROS_WS} \
#    && catkin_make

## Overlay the ROS environment on top of the existing environment
#COPY docker/drone/ros_entrypoint.sh /
#RUN ["chmod", "+x", "/ros_entrypoint.sh"]
#ENTRYPOINT ["/ros_entrypoint.sh"]
#CMD ["bash"]