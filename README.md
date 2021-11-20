# NIMBUS Docker Images
This repository hosts a collection of Docker images and docker-compose files that can be used to launch the respective images. To get started with this repository, refer to [Getting Started](#getting-started). For more comprehensive documentation, please refer to the project Wiki.


## Getting Started
The following section serves to document the high-level steps to utilize and integrate the NIMBUS Docker images into your respective project. For more in-depth information, please refer to the project Wiki.


### Requirements
Prior to getting started with this project, ensure that the following software requirements are met:
* [Docker](https://docs.docker.com/get-docker/)
* [docker-compose](https://docs.docker.com/compose/install/)
* Login to the NIMBUS container registry:
    * Generate a [Personal Access Token](https://docs.github.com/en/authentication/keeping-your-account-and-data-secure/creating-a-personal-access-token) (PAT)
    * Copy the generated token
    * `docker login ghcr.io`
        * When prompted to enter your password, insert the generated PAT


### Pulling a Docker Image
To pull a Docker image from this repository refer to the steps below. Note that, for images that provide `docker-compose.yaml` scripts, this step is not necessary.
1. Navigate to the `unl-nimbus-lab/docker` repository
2. Click on the repository `Packages`
    * **NOTE:** This will display a list of all released images
3. Click on the desired image to pull
4. Copy the installation command provided
    * **NOTE:** The provided command should look similar to the following command:
        * `docker pull ghcr.io/unl-nimbus-lab/docker/my-image`
5. Execute the command copied within the previous step on your local system.
    * **NOTE:** This should pull the desired image to your local Docker images for usage


### Usage
The Docker images provided within this repository may be used in a couple of different ways: using the provided `docker-compose.yaml` files, running the image interactively, or running the image as single-shot container. The method that you take to utilize a given image is subjective to its design. Full documentation regarding the usage of each image is provided within the respective image's


There are two methods that images within this project may be utilized: using the released images hosted on this repository interactively or using `docker-compose`. The distinguishing factor regarding the usage of these images is their purpose and the availablity of a `docker-compose.yaml` file within the respective image's directory. For example, should a User desire to run a Docker container interactively (e.g., the `development` image), it will be necessary to pull the desired image from this repository. However, if a User desires to run the as a service or collection of services (e.g., the vicon image) it will be necessary to utilize `docker-compose`. Note that the images that utilize `docker-compose` may still be run interactively; however, images that do not provide a `docker-compose.yml` file may not utilize `docker-compose`. For more information regarding each of the provided Docker images, please refer to the project Wiki. To quickly get started with using images interactively, refer to the steps below:

* Running an image interactively:
    1. After successfully pulling or building a Docker image, the image can then be run interactively. That is, the image can be run with an interactive bash environment similar to a Linux Terminal. To run an image interactively, execute the following command:
        * `docker run -it my-image`

### Using docker-compose
To utilize `docker-compose` for your Docker images refer to the steps below:
1. Navigate to the `unl-nimbus-lab/docker` repository
2. Clone the project repository:
    * `git clone https://github.com/unl-nimbus-lab/docker.git`
3. Navigate to the desired image's directory
    * `cd path/to/docker/my-image`
4. Use `docker-compose` to pull the desired image and run the services:
    * `docker-compose up`
    * **NOTE:** To run `docker-compose` as a daemon, use following command:
        * `docker-compose up -d`
5. To shutdown the `docker-compose` services, run the following command:
    * `docker-compose down`