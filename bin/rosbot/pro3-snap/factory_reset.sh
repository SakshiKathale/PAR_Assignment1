#!/bin/bash

# Check if the script is being run as a normal user
if [ "$(id -u)" -eq 0 ]; then
    echo "Error: This script must be run as a normal user."
    exit 1
fi

SNAP_LIST=(
    rosbot
    husarion-rplidar
    husarion-depthai
    husarion-webui
)

ROS_DISTRO=${ROS_DISTRO:-humble}

start_time=$(date +%s)

/var/snap/rosbot/common/manage_ros_env.sh remove
sudo /var/snap/rosbot/common/manage_ros_env.sh remove

for snap in "${SNAP_LIST[@]}"; do
    echo "---------------------------------------"
    echo "removing the \"$snap\" snap"
    sudo snap remove "$snap" 
done

for snap in "${SNAP_LIST[@]}"; do
    echo "---------------------------------------"
    echo "Installing the \"$snap\" snap (ROS 2 $ROS_DISTRO)"
    sudo snap install "$snap" --channel="$ROS_DISTRO"
    sudo "$snap".stop
    sudo snap set "$snap" \
        ros.transport=udp-lo \
        ros.localhost-only='' \
        ros.domain-id=0 \
        ros.namespace=''
done

echo "---------------------------------------"
echo "Setting up the \"rosbot\" snap"
sudo snap connect rosbot:shm-plug rosbot:shm-slot
sudo /var/snap/rosbot/common/post_install.sh
sudo rosbot.stop
sleep 2
sudo rosbot.flash

echo "---------------------------------------"
echo "Setting up the \"husarion-rplidar\" snap"
sudo snap connect husarion-rplidar:shm-plug husarion-rplidar:shm-slot
sudo snap set husarion-rplidar configuration=s2

echo "---------------------------------------"
echo "Setting up the \"husarion-depthai\" snap"
sudo snap connect husarion-depthai:shm-plug husarion-depthai:shm-slot
sudo snap set husarion-depthai driver.parent-frame=camera_link

echo "---------------------------------------"
echo "Setting up the \"husarion-webui\" snap"
sudo cp foxglove-rosbot3.json /var/snap/husarion-webui/common/
sudo snap set husarion-webui webui.layout=rosbot3

echo "---------------------------------------"
echo "Default DDS params on host"
/var/snap/rosbot/common/manage_ros_env.sh
sudo /var/snap/rosbot/common/manage_ros_env.sh

echo "---------------------------------------"
echo "Start all snap"

for snap in "${SNAP_LIST[@]}"; do
    sudo "$snap".start
    # sudo "$snap".restart
done

end_time=$(date +%s)
duration=$(( end_time - start_time ))

hours=$(( duration / 3600 ))
minutes=$(( (duration % 3600) / 60 ))
seconds=$(( duration % 60 ))

printf "Script completed in %02d:%02d:%02d (hh:mm:ss)\n" $hours $minutes $seconds

