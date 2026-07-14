#! /bin/bash

PX4_DOCKER_REPO="px4-qgc:px4-v1.16.2"

# Help function
show_help() {
	echo "Usage: $0 [OPTIONS] [COMMAND]"
	echo ""
	echo "Run PX4 development Docker container."
	echo ""
	echo "Options:"
	echo "  --ip <ADDRESS>    Set container IP address and name container as px4-v1.16.2_<ADDRESS>"
	echo "  --help            Show this help message"
	echo ""
	echo "Examples:"
	echo "  $0                                    # Run container with name 'px4-v1.16.2'"
	echo "  $0 --ip 192.168.1.100                 # Run with IP and name 'px4-v1.16.2_192.168.1.100'"
	echo "  $0 --ip 192.168.1.100 make px4_sitl   # Run with IP and execute command"
	exit 0
}

# Parse arguments
IP_ADDRESS=""
while [[ $# -gt 0 ]]; do
	case $1 in
		--ip)
			IP_ADDRESS="$2"
			shift 2
			;;
		--help)
			show_help
			;;
		*)
			break
			;;
	esac
done

# Set container name based on IP address
if [ -n "$IP_ADDRESS" ]; then
	CONTAINER_NAME="px4-v1.16.2_${IP_ADDRESS}"
else
	CONTAINER_NAME="px4-v1.16.2"
fi

# docker hygiene

#Delete all stopped containers (including data-only containers)
#docker rm $(docker ps -a -q)

#Delete all 'untagged/dangling' (<none>) images
#docker rmi $(docker images -q -f dangling=true)

echo "PX4_DOCKER_REPO: $PX4_DOCKER_REPO";
echo "CONTAINER_NAME: $CONTAINER_NAME";

PWD=$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )
HOST_SRC_DIR=$PWD/../
CONTAINER_SRC_DIR=/home/ubuntu/PX4-Autopilot

CCACHE_DIR=${HOME}/.ccache
mkdir -p "${CCACHE_DIR}"

# Build docker run command
DOCKER_CMD="docker run -it --rm -w \"${CONTAINER_SRC_DIR}\" \
	--name=\"${CONTAINER_NAME}\" \
	--user=\"$(id -u):$(id -g)\" \
	--network fw_swarm_local"

if [ -n "$IP_ADDRESS" ]; then
	DOCKER_CMD="${DOCKER_CMD} \
	--ip=${IP_ADDRESS}"
fi

DOCKER_CMD="${DOCKER_CMD} \
	--env=BRANCH_NAME \
	--env=CCACHE_DIR=\"${CCACHE_DIR}\" \
	--volume=${CCACHE_DIR}:${CCACHE_DIR}:rw \
	--volume=${HOST_SRC_DIR}:${CONTAINER_SRC_DIR}:rw \
	${PX4_DOCKER_REPO} /bin/bash -c \"$@\""

eval $DOCKER_CMD
