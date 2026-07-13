#! /bin/bash

PX4_DOCKER_REPO="px4-qgc:px4-v1.16.2"

# docker hygiene

#Delete all stopped containers (including data-only containers)
#docker rm $(docker ps -a -q)

#Delete all 'untagged/dangling' (<none>) images
#docker rmi $(docker images -q -f dangling=true)

echo "PX4_DOCKER_REPO: $PX4_DOCKER_REPO";

PWD=$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )
HOST_SRC_DIR=$PWD/../
CONTAINER_SRC_DIR=/home/ubuntu/PX4-Autopilot

CCACHE_DIR=${HOME}/.ccache
mkdir -p "${CCACHE_DIR}"

docker run -it --rm -w "${CONTAINER_SRC_DIR}" \
	--user="$(id -u):$(id -g)" \
	--network fw_swarm_local \
	--env=BRANCH_NAME \
	--env=CCACHE_DIR="${CCACHE_DIR}" \
	--volume=${CCACHE_DIR}:${CCACHE_DIR}:rw \
	--volume=${HOST_SRC_DIR}:${CONTAINER_SRC_DIR}:rw \
	${PX4_DOCKER_REPO} /bin/bash -c "$@"
