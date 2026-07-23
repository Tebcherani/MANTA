# Installing Docker on Ubuntu

This guide explains how to install Docker Engine on Ubuntu using Docker's official package repository. After completing these steps, you will be able to build Docker images, run Docker containers, and use Docker without requiring `sudo`.

## Prerequisites

Update the package index and install the packages required to access HTTPS repositories.

```bash
sudo apt update
sudo apt install ca-certificates curl
```

## Create the Docker Keyring Directory

Create the directory that will store Docker's repository signing key.

```bash
sudo install -m 0755 -d /etc/apt/keyrings
```

## Download Docker's GPG Key

Download Docker's official GPG key and make it readable by all users.

```bash
sudo curl -fsSL https://download.docker.com/linux/ubuntu/gpg -o /etc/apt/keyrings/docker.asc
sudo chmod a+r /etc/apt/keyrings/docker.asc
```

## Add the Docker Repository

Add Docker's official package repository to your system.

```bash
echo "deb [arch=$(dpkg --print-architecture) signed-by=/etc/apt/keyrings/docker.asc] https://download.docker.com/linux/ubuntu $(. /etc/os-release && echo "${UBUNTU_CODENAME:-$VERSION_CODENAME}") stable" | sudo tee /etc/apt/sources.list.d/docker.list > /dev/null
```

## Update the Package Index

Refresh the package list so Ubuntu recognizes the newly added Docker repository.

```bash
sudo apt update
```

## Install Docker

Install Docker Engine, the Docker CLI, Buildx, and Docker Compose.

```bash
sudo apt install docker-ce docker-ce-cli containerd.io docker-buildx-plugin docker-compose-plugin
```

When prompted, type `Y` to continue.

## Verify the Installation

Confirm that Docker and Docker Compose were installed successfully.

```bash
docker --version
docker compose version
```

Both commands should print a version number.

## Test Docker

Run Docker's test container.

```bash
sudo docker run hello-world
```

If Docker is working correctly, the output will contain the following message:

> This message shows that your installation appears to be working correctly.

## Enable Non-Root Docker Access

By default, Docker commands require `sudo`. To allow your user account to run Docker directly, add your user to the `docker` group.

```bash
sudo usermod -aG docker $USER
newgrp docker
```

## Verify Non-Root Access

Run the test container again without `sudo`.

```bash
docker run hello-world
```

If the container runs successfully, Docker has been configured correctly.

## Clean Up

### List Existing Containers

```bash
docker ps -a
```

### Remove the Hello World Containers

There will typically be two stopped `hello-world` containers.

```bash
docker rm <container_id>
```

Repeat this command for each container.

Verify that they have been removed.

```bash
docker ps -a
```

### List Existing Images

```bash
docker images
```

### Remove the Hello World Image

```bash
docker rmi <image_id>
```

Verify that the image has been removed.

```bash
docker images
```

## Next Steps

Now that Docker is installed, you can:

- Build Docker images using `docker build`
- Run containers using `docker run`
- Use bind mounts to share files between your host machine and Docker containers
- Create reusable workflows with shell scripts
- Build multi-container applications using Docker Compose