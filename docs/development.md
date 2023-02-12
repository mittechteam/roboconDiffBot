# Development Using Docker

This document describes how to develop using Docker.

## Prerequisites

- [Docker](docs/dockerInstallation.md) : Installation of Docker & authentication with a PAT (Personal Access Token)

---

## Docker Container for Development

### Run

Run the Docker container:

```bash
$ docker run -it -d --name mttros2 -v $(pwd):/app -w /app -p 8080:8080 ghcr.io/mittechteam/mtt-ros2-dev
```

Open the browser and go to http://localhost:8080/vnc.html.

---

## Compiling ROS2 workspace

inside the docker container open a terminal and run the following commands:

### Build

Build the ROS2 workspace:

```bash
$ colcon build --symlink-install
```

### Source

Source the ROS2 workspace:

```bash
$ source install/setup.bash
```

### Run

Run the ROS2 workspace:

```bash
$ ros2 run <package_name> <executable_name>
```
