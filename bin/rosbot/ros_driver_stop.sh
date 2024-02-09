#!/bin/bash

# Get the current user
CURRENT_USER=$(whoami)

# Check if the current user is "husarion"
if [ "$CURRENT_USER" != "husarion" ]; then
  echo "This script can only be run by the user 'husarion'."
  exit 1
fi

# Define color codes
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
RED='\033[0;31m'
BOLD='\033[1m'
NC='\033[0m' # No Color

# Stop the Docker containers if they're running
echo -e "${GREEN}[1/1]\r\nStopping ROS 2 driver ${NC}"

# Define the docker-compose file
COMPOSE_FILE="/home/husarion/compose.yaml"

# This is a temporary solution allowing shared memory communication between 
# host and docker container. To be removed when user will be able to change this permission
# to something else than 0644 (https://github.com/eProsima/Fast-DDS/blob/master/thirdparty/boost/include/boost/interprocess/permissions.hpp#L100) 
# You need to start containers first, after that new files in /dev/shm/ are created. We need to change their permissions to 0666
export DOCKER_UID=$(id -u husarion)
export DOCKER_GID=$(id -g husarion)

docker compose -f $COMPOSE_FILE down
ros2 daemon stop

echo -e "${GREEN}done.${NC}"
