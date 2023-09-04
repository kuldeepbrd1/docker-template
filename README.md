# Docker template for containerized projects

TODO: Non-root user

## Background

I have used docker only occasionally and for less than 2 years. But, I've quickly grown fond of it, especially for using containers during development. For robotics+ML stuff that I do, this has been a blessing.

The idea of this template repo is that copying these files directly into a new project should give you a quick start to add docker support for your development and release code.

I've slowly been building up rules of thumb for my projects that now benefit from a common(ish) structure. This repo consolidates this effort. This is mainly for my personal use. But, if you find it helpful, feel free to use it. If you have suggestions for improvements, feel free to open an issue or a PR.

## Pre-requisites

- Docker
- VSCode (If using)
  - VSCode Remote Containers extension

## Components

### Devcontainer

```shell
.
├── ./.devcontainer                             # This is where vscode looks for a devcontainer.json file
│   └── ./.devcontainer/devcontainer.json       # This file defines the container setup
```

VSCode Devcontainers help me setup containers quickly during development and iterate rapidly. Especially when a project requires messing with system libraries or which is not supported on the host OS (ahem, ROS1), developing inside containers is the only sane way to go.

So, I prefer to use VSCode devcontainers during development quite often. You get the familiar VS Code IDE and all its extensions while developing inside the container. One my favorite things that devcontainers offer is native experience in debugging. But there is a lot more. For more information, see [here](https://code.visualstudio.com/docs/remote/containers).

In principle, it spins a container based on the same image/Dockerfile that you normally use, but installs vscode server and other tools that you need to communicate with the container and enable remote development.

### Dockerfile and Helper Scripts

```shell
├── ./.docker                                   # Docker: This is where we consolidated docker related files in the repo
│   ├── ./.docker/development.Dockerfile        # Development Dockerfile
│   ├── ./.docker/docker_build.bash             # Helper script to build docker image
│   ├── ./.docker/docker-compose.yml            # Docker compose file for multi-container projects
│   ├── ./.docker/docker_run.bash               # Helper script to run docker container
│   ├── ./.docker/entrypoint.sh                 # Entrypoint script for the container
│   └── ./.docker/release.Dockerfile            # Release Dockerfile

```

I consolidate Dockerfiles and helper scripts all in `.docker` directory.

In the template, there are two Dockerfiles: [`development.Dockerfile`](.docker/development.Dockerfile) and [`release.Dockerfile`](.docker/release.Dockerfile).

They outline some details on how development and release images can differ a little bit. Each of them contains detailed comments that should be self-explanatory.

I use helper scripts [`docker_build.bash`](.docker/docker_build.bash) and [`docker_run.bash`](.docker/docker_run.bash) to build and run the containers. They require minimal change (the image name) to work with another project.

```shell
# Build the image
./.docker/docker_build.bash

# Pass custom arguments to docker build
./.docker/docker_build.bash --build-arg CUDA_VERSION=11.8.1 <...>

# Run the container
./.docker/docker_run.bash

# Pass custom arguments to docker run
./.docker/docker_run.bash --name my_container --no-cache <...>
```

For multi-container projects, Docker compose is great. I have not used it a lot. But, [`docker-compose.yml`](.docker/docker-compose.yml) has worked well for me so far in ML+ROS projects. It defines a boilerplate with a ROS container and GPU enabled ML application.

```shell
# Run the containers
docker-compose up

# Run the containers in the background
docker-compose up -d

# Build and run the containers
docker-compose up --build
```

### Dockerignore

Do not add everything to the docker context. Ignore everything that is not needed during build, so you don't waste time or image size.

```shell
├── ./.dockerignore                             # Dockerignore: This is where we consolidated dockerignore files in the repo
```

### Pre-commit

Pre-commit is a great tool to run code quality and beauty checks before you commit your code. I only recently learned about it but I'm liking it so far. It is easy to setup and use. The important part is the hadolint check, which checks for best practices in Dockerfiles.

```shell
├── ./.pre-commit-config.yaml                   # Pre-commit: This is where we consolidated pre-commit config files in the repo
```

To use pre-commit hooks, make sure you have pre-commit installed. Then, run `pre-commit install` in the project root.:

```shell
# Install pre-commit
pip install pre-commit

# Install pre-commit hooks to .git/hooks/pre-commit
cd <project_root>
pre-commit install
```

Once installed, before every commit, it will run configure code quality checks.
