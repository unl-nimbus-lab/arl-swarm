# syntax=docker/dockerfile:1
FROM ubuntu:20.04

# Setup the environments
ENV LANG=C.UTF-8
ENV LC_ALL=C.UTF-8
ENV ROS_DISTRO=noetic
ENV HOME=/home/
ENV ROS_WS=/home/catkin_ws/
WORKDIR ${ROS_WS}

RUN apt-get -y update && apt-get -y install software-properties-common && \
    add-apt-repository -y ppa:deadsnakes/ppa && apt install -y python3.10


# Install utilities/dependencies
RUN apt-get update && apt-get install -y --no-install-recommends \
        iputils-ping \
        net-tools \
        openssh-server \
        netbase \
        python3 \
        python3-pip \
        git \
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
        build-essential \
        lsb-release \
        ca-certificates \
        dirmngr \
        gnupg2 \
        python3 \
        python3-pip \
        python3-dev \
        netbase \
        htop \
        nmap \
    && rm -rf /var/lib/apt/lists/*

COPY Utilities/bag-manager.py /home/