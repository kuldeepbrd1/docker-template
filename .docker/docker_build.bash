#!/bin/bash

ORG_NAME="kuldeepbrd1"
IMAGE_NAME="ros-uois"
VERSION="1.0.0"
IMAGE_TAG="${ORG_NAME}/${IMAGE_NAME}:${VERSION}"


# Get current file directory
CURRENT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" >/dev/null 2>&1 && pwd )"
REPO_DIR="$(dirname "$CURRENT_DIR")"

DOCKERFILE="${CURRENT_DIR}/uois-ros-realsense.Dockerfile"

docker build -f ${DOCKERFILE} -t ${IMAGE_TAG} "$@" ${REPO_DIR}

LATEST_TAG=${ORG_NAME}/${IMAGE_NAME}:latest
docker tag ${IMAGE_TAG} ${LATEST_TAG}
