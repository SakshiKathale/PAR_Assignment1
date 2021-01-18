#!/bin/bash
# It is required to make below modifications in ROSbot system
# Reboot after executing script is necessary

function download_greengrass () {
    # check system architecture
    ARCH=$(uname -m)
    if [ "$ARCH" == "x86_64" ]; then
        echo "Working on x86_64"
        sudo wget https://raw.githubusercontent.com/husarion/rosbot-robomaker/master/rosbot_pro.yaml -O /rosbot_conf.yaml
        GREP_EXPR="greengrass-linux-x86-64"
        if ! grep -q 'ROSBOT_VER="2.0_PRO"' /etc/environment; then
            echo 'ROSBOT_VER="2.0_PRO"' >> /etc/environment
        fi
    elif [ "$ARCH" == "armv7l" ]; then
        echo "Working on armv7l"
        sudo wget https://raw.githubusercontent.com/husarion/rosbot-robomaker/master/rosbot_2_0.yaml -O /rosbot_conf.yaml
        GREP_EXPR="greengrass-linux-armv7l"
        if ! grep -q 'ROSBOT_VER="2.0' /etc/environment; then
            echo 'ROSBOT_VER="2.0"' >> /etc/environment
        fi
    else 
        echo "No compatible architecture detected"
        return 1
    fi

    # Get Greengrass donload page
    GG_HTML=$(wget -O - https://docs.aws.amazon.com/greengrass/latest/developerguide/what-is-gg.html)
    # Extract download links
    LINK_LINES=$( echo "$GG_HTML" | grep $GREP_EXPR)

    # download file
    IFS=$'\n'
    for HTML_LINE in $LINK_LINES
    do
        DL_LINK=$(expr match "$HTML_LINE" '.*\(https://.*tar.gz\)')
        echo "Downloading from: $DL_LINK"
        wget $DL_LINK -O greengrass.tar.gz
        if [ $? -eq 0 ]; then
            echo "Download complete!"
            return 0
            break
        fi
    done
    return 1
}

if [ "$(id -u)" != 0 ]; then
    echo "Error: please run the installer as root."
    exit 1
fi

echo 'deb http://ppa.launchpad.net/webupd8team/java/ubuntu xenial main
# deb-src http://ppa.launchpad.net/webupd8team/java/ubuntu xenial main' > /etc/apt/sources.list.d/webupd8team-ubuntu-java-xenial.list

apt-key adv --keyserver keyserver.ubuntu.com --recv-keys EEA14886
sudo apt-key adv --keyserver keyserver.ubuntu.com --recv-keys C2518248EEA14886

apt update
curl -sL https://deb.nodesource.com/setup_6.x | bash -
apt install -y nodejs software-properties-common python3-colcon-common-extensions python3-pip jq
pip3 install --upgrade colcon-common-extensions setuptools
ln /usr/bin/node /usr/bin/nodejs6.10

echo 'KERNEL=="ttyUSB*", ATTRS{idVendor}=="10c4", ATTRS{idProduct}=="ea60", MODE:="0777", SYMLINK+="rplidar"' > /etc/udev/rules.d/rplidar.rules


echo 'KERNEL=="ttyUSB1", MODE:="0777", SYMLINK+="core2serial"
KERNEL=="ttyCORE2", MODE:="0777"' > /etc/udev/rules.d/core2serial.rules

adduser --disabled-password --gecos "" ggc_user
addgroup --system ggc_group

if ! grep -q 'fs.protected_hardlinks = 1' /etc/sysctl.d/98-rpi.conf; then
    echo 'fs.protected_hardlinks = 1' >> /etc/sysctl.d/98-rpi.conf
fi

if ! grep -q 'fs.protected_symlinks = 1' /etc/sysctl.d/98-rpi.conf; then
    echo 'fs.protected_symlinks = 1' >> /etc/sysctl.d/98-rpi.conf
fi

if ! grep -q 'cgroup_enable=memory cgroup_memory=1' /boot/cmdline.txt; then
    LINE=$(sed 's/$/ cgroup_enable=memory cgroup_memory=1/' /boot/cmdline.txt)
    echo $LINE > /boot/cmdline.txt
fi

LINE='if [ -f /tmp/roboMakerDeploymentPackage/packageInfo ] ; then for f in $(jq ".overlays | .[]" /tmp/roboMakerDeploymentPackage/packageInfo -r); do echo $f; BUNDLE_CURRENT_PREFIX=/tmp/roboMakerDeploymentPackage/$f; source /tmp/roboMakerDeploymentPackage/$f/setup.sh --extend; done; fi'
if ! grep -q '/tmp/roboMakerDeploymentPackage/packageInfo' ~/.bashrc; 
then 
    echo $LINE >> ~/.bashrc
fi

# download_greengrass
# if [ $? -eq 0 ]; then
#     echo "Function executed successfully"
# else 
#     echo "Can not download Greengrass, please check your internet connection."
# fi
tar -zxvf greengrass-linux-armv7l-1.9.4.tar.gz -C /

cd /greengrass/certs
wget -O root.ca.pem https://www.amazontrust.com/repository/AmazonRootCA1.pem

source /opt/ros/kinetic/setup.bash
mkdir ~/ros_workspace
mkdir ~/ros_workspace/src
cd ~/ros_workspace/src
catkin_init_workspace
cd ~/ros_workspace
catkin_make
echo "source ~/ros_workspace/devel/setup.sh" >> ~/.bashrc
source ~/ros_workspace/devel/setup.sh
cd ~/ros_workspace/src/
git clone https://github.com/husarion/tutorial_pkg.git
cd ~/ros_workspace/
rosdep install --from-paths src --ignore-src -r -y
catkin_make

echo "You ned to restart ROSbot now to apply all changes"
