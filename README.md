# arl-swarm

> **Paper:** G. Phillips, H.-D. Tran, and J. M. Bradley, "Robust Safety Assurance for Robotic Swarms Using Decentralized Collision Verification," *IEEE Transactions on Aerospace and Electronic Systems*, vol. 62, pp. 9706-9717, 2026. DOI: [10.1109/TAES.2026.3683620](https://doi.org/10.1109/TAES.2026.3683620). The `verifier_2025` branch is the code used in that paper; the runtime verifier (`safety_checker`) is in `swarm-nodes/verifier/`.

## Build the development image

`docker build -f docker/development/Dockerfile -t swarm_devel .`

### Check the Utilities folder for the command to start the development image

## Build the simulator image

`docker build -f SimulatorDockerfile -t hrl_docker .`

## use the gcs

'docker run -it --net=host hrl_docker'

'rosrun verifier_comms gcs_backend.py'

## Introduction
This repository contains the implementation, tools, and processes that have been used by the ARL Swarm Team for swarm deployment. For more comprehensive documentation, please refer to the project Wiki.

## Installation
Prior to installing this project, it is necessary to first install the software requirements listed below. Note that these requirements must also be met on the respective agents.

* [Docker](https://docs.docker.com/get-docker/)
* [docker-compose](https://docs.docker.com/compose/install/)
* Login to the NIMBUS container registry:
    * Generate a [Personal Access Token](https://docs.github.com/en/authentication/keeping-your-account-and-data-secure/creating-a-personal-access-token) (PAT)
    * Copy the generated token
    * `docker login ghcr.io`
        * When prompted to enter your password, insert the generated PAT

Once these software requirements have been met, it is now necessary to clone the project repository. Performing this step will provide access to the project's `docker-compose.yml` scripts and helper scripts.

```
git clone https://github.com/unl-nimbus-lab/docker.git
```

After cloning the project repository, it is now possible to install and deploy the Docker images provided within this project. The provided Docker images contain all code and the respective environments so that Users may continue to develop and deploy this project.


## Getting Started
The `arl-swarm` repository provides two Docker images: `arl-swarm/drone` and `arl-swarm/development`. The `drone` image contains the implementation and environment to deploy the swarm software stack on a drone. The `development` image contains the implementation and environment to develop software for the swarm.

### Launching an `arl-swarm/development` Container
The `arl-swarm/development` image comes with a helper script (`start.sh`) to improve usability of the container and to help new Users get started with the project:

```bash
# Navigate to the correct directory
cd arl-swarm/docker/development/scripts

# Pull the latest image
./start.sh -p

# See the list of available commands
./start.sh -h
```

For Users familiar with Docker that wish to interact directly with the image, you may use the `start.sh` script to pull the latest version of the image and utilize the downloaded image as you would any other image.


### Launching an `arl-swarm/drone` Container
Prior to launching the `arl-swarm/drone` container, it is first necessary to modify the environment variables provided within the `.env` file and to source this environment file:

```bash
# Navigate to the proper directory
cd path/to/arl-swarm/docker/drone/

# Modify the BAUD, PORT, SYS_ID, and COMP_ID variables to the desired values and save the file
nano .env

# Source the environment
source .env
```

After setting the desired environment variable values and sourcing the environment, the `docker-compose.yml` script may be used to launch the container.

```bash
# Navigate to the proper directory
cd path/to/arl-swarm/docker/drone/

# Run the docker services
docker-compose up
```

CI TEST