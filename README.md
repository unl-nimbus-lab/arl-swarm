# ros-nodes
This repository hosts a collection of commonly used ROS nodes. To get started with this repository, refer to [Getting Started](#getting-started). For more comprehensive documentation regarding this project please refer to the project Wiki.

## Getting Started
The following section serves to document the high-level steps to utilize and integrate the NIMBUS ROS nodes into your respective project. For more in-depth information, please refer to the project Wiki.

### Requirements
Prior to getting started with this project, ensure that the following software requirements are met:
* [ROS Noetic](http://wiki.ros.org/noetic/Installation)


### Installation
For Users of the NIMBUS Docker images, (e.g., the `development` image), it is not necessary to install these ROS nodes. The images ship with these ROS nodes by default. For all other users, to install the provided ROS nodes, clone this repository to the `src/` directory of your ROS workspace. Please note that, the most recent stable versions of the ROS nodes are located within the `main` branch. For the latest versions of the images, which are unstable, please checkout the `develop` branch. After cloning the repository, you should have a directory structure similar to the following:

```bash
├── catkin_ws/
│   ├── build/
│   ├── devel/
│   ├── src/
└── └── └── ros-nodes/
```

After successfully installing these ROS nodes, navigate to the base directory of your ROS workspace (i.e., `catkin_ws`), and build the packages using `catkin_make` or `catkin build`. After building the packages, it may be necessary to source the workspace environment (if this was not done within the `.bashrc` file):
```
source catkin_ws/devel/setup.bash
```


### Usage
To utilize the implemented ROS nodes, each node has implemented a launch script. Therefore, to utilize the desired node, `roslaunch` may be used. For example:

```
roslaunch land land.launch
```

For more comprehensive information regarding the usage of the launch files and their respective nodes, please refer to the project Wiki.
