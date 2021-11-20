# syntax=docker/dockerfile:1
FROM ros:noetic

# Create the image file structure
RUN mkdir /home/catkin_ws \
    && mkdir /home/catkin_ws/src

# Setup the environments
ENV LANG=C.UTF-8
ENV LC_ALL=C.UTF-8
ENV ROS_DISTRO=noetic
ENV HOME=/home/
ENV ROS_WS=/home/catkin_ws
WORKDIR ${ROS_WS}

# Install dependencies/utilities
RUN apt-get update && apt-get install -y --no-install-recommends \
    git \
    nano \
    ros-noetic-tf \
    ros-noetic-diagnostic-updater \
    && rm -rf /var/lib/apt/lists/*

# Clone the Vicon repository
RUN git clone https://github.com/ethz-asl/vicon_bridge.git \
    && mv vicon_bridge /home/catkin_ws/src

# Build the ROS workspace
RUN . /opt/ros/${ROS_DISTRO}/setup.sh \
    && cd ${ROS_WS} \
    && catkin_make

# Overlay the ROS environment on top of the existing environment
COPY ./ros_entrypoint.sh /
RUN ["chmod", "+x", "/ros_entrypoint.sh"]
ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["bash"]