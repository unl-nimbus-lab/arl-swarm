pipeline {
    agent any

    environment {
        // Set the Docker image name
        DOCKER_IMAGE_NAME = 'grantphllps/arl_swarm'
    }

    stages {
        stage('Checkout') {
            steps {
                // Clone the GitHub repository
                git url: 'https://github.com/unl-nimbus-lab/arl-swarm.git'
            }
        }
        stage('Build Docker Image') {
            steps {
                script {
                    // Set the Docker image name and tag
                    def imageName = "${DOCKER_IMAGE_NAME}"
                    def imageTag = "latest"

                    // Build the Docker image
                    sh "docker build -f SimulatorDockerfile -t ${imageName}:${imageTag} ."
                }
            }
        }
    }
}