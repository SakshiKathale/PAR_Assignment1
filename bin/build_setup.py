#! /usr/bin/env python3

'''
build_setup:
- Configures the build environment for working with ROSBot and ROS Setup in the AIIL

@author
- Timothy Wiley
'''

import argparse
import os
import pathlib
import re
import shutil
import stat
import subprocess
import sys

import utils.config as cfg
from utils.echo import (
    print_error,
    print_status,
    print_warning,
    print_progress,
    print_subitem,
    query_user,
    query_yes_no
)
from utils.grep import (
    grep
)
import utils.shellEscape as shell
import utils.snapinstall as snap
import utils.sshscp as ssh
import utils.wireless as wireless

# Global Parameters
fullSetupPath = os.path.abspath(__file__)
binDir = os.path.dirname(fullSetupPath)
configDir = os.path.abspath(binDir + "/../config/")
AIIL_CHECKOUT_DIR = os.path.abspath(binDir + "/../")
HUSARION_CHECKOUT_DIR = os.path.abspath(AIIL_CHECKOUT_DIR + "/humble_workspace/src/" + cfg.husarion_workspace)

# Global setup type parameters
setupRobot      = False
setupRobotName  = None
setupComp       = False
setupCompName   = None
setupTheConst   = False
setupDocker     = False

def installSoftware(software, ros=False, rosversion='none', skipcheck=False):
    # Ensure NPM is available
    if not ros:
        if False:
            print_status("Setup NPM package location")
            command = "curl -sL https://deb.nodesource.com/setup_12.x | sudo -E bash -"
            print_subitem("Running: " + command)
            shell.exec(command, hideOutput=False)
        else:
            print_status("NPM not required - disabled")

    # Load this here to avoid script delays
    print_status("Installing Software")
    from utils.aptinstall import (aptinstall, aptcheck, aptinstallcmdline)

    toInstall = []

    # Load Software to install
    counter = 0
    for pkg in software:
        progress = round(counter / len(software) * 100)

        if software.getboolean(pkg):
            # If ROS package - prepend with ros package name
            if ros:
                pkg = "ros-" + rosversion + "-" + pkg

            print_progress("Installing " + pkg, progress)
            installed = aptcheck(pkg)
            if not skipcheck and installed:
                print_progress("Package " + pkg + " installed. Skipping", progress)
            else :
                print_progress("Marking " + pkg + " for installation", progress)
                toInstall.append(pkg)
        else :
            print_progress("Skipping package - configuration file disables install of " + pkg, progress)
        counter += 1
    print_progress("Installation checked", 100)

    if len(toInstall) > 0:
        print_warning("Installation will most likely request admin privillages to complete installation")
        print_status("Commencing Installation")
        print_subitem("Packages: " + " ".join(toInstall))
        aptinstallcmdline(toInstall)
    else :
        print_subitem("No packages to install")

def installSnap(configRobots, configSnap):
    print_status("Configuring Snap")

    # Fix snapd install for Pro2
    if configRobots[setupRobotName]['husarion'] == 'pro2':
        # Unmask snapd units
        shell.exec('sudo systemctl unmask snapd.service')
        shell.exec('sudo systemctl unmask snapd.socket')
        shell.exec('sudo systemctl unmask snapd.seeded.service')

        # Enable units to start automatically
        shell.exec('sudo systemctl enable snapd.service')
        shell.exec('sudo systemctl enable snapd.socket')
        shell.exec('sudo systemctl enable snapd.seeded.service')

        # Start snapd
        shell.exec('sudo systemctl start snapd.service')
        shell.exec('sudo systemctl start snapd.socket')
        shell.exec('sudo systemctl start snapd.seeded.service')
    
    query = query_yes_no("Install Snap package(s)")
    if query:
        # Remove existing snap (which causes conflict)
        if setupRobot:
            toRemove = [
                'husarion.rosbot'
            ]
            for pkg in toRemove:
                snap.remove_snap_package(pkg)
        
        # Install Snap packages
        for pkg in configSnap:
            # Either the channel to install, true or false
            pkgValue = configSnap[pkg]
            if pkgValue != 'False':
                channel = None
                if pkgValue != 'True':
                    channel = pkgValue
                snap.install_snap_packages(pkg, channel=channel)
            else :
                print_subitem("Skipping package - configuration file disables install of " + pkg)
    print()
    
    # Configure Snap packages
    query = query_yes_no("Configure Snap package properties")
    if query:
        if setupRobot:
            ip = configRobots[setupRobotName]["ip"]
            snap.configure_snap_property("husarion-webui", "ros.domain-id", ip)
        
        # snap.configure_snap_property("husarion-webui", "ros.transport", "rmw_cyclonedds_cpp")
        # snap.configure_snap_property("husarion-webui", "ros.transport", "rmw_fastrtps_cpp")
        snap.configure_snap_property("husarion-webui", "ros.transport", "udp")
    print()   

def replaceAuthKeys():
    localAuthFile = ssh.authKeyFile()
    target = ssh.sshDirectory
    shutil.copy(localAuthFile, target)

def setupBash():
    print_status("Configuring Bash Environment")
    rbbBashFile = os.path.abspath(AIIL_CHECKOUT_DIR + "/.bashrc")
    bashrcFile = os.path.expanduser("~/.bashrc")
    
    # Find required config files
    configDir = os.path.abspath(AIIL_CHECKOUT_DIR + "/config")
    configRobots = cfg.loadConfigFile(configDir + "/robots.cfg")
    
    # Create redbackbots source file
    templateBashrc = os.path.abspath(AIIL_CHECKOUT_DIR + "/config/bashrc")
    shutil.copy(templateBashrc, rbbBashFile)
    print_subitem("Copying " + templateBashrc + " to " + rbbBashFile)

    # Set Husarion workspace based on device type
    husDir = HUSARION_CHECKOUT_DIR
    # if setupRobot:
        # husDir = os.path.expanduser("~/" + cfg.husarion_workspace)
    
    # Add ROSBot env variables to start of the bashrc
    with open(rbbBashFile, "r+") as bashFile:
        content = bashFile.read()
        bashFile.seek(0, 0)
        
        bashFile.write("# ROSBot Environment Settings\n")
        bashFile.write("export AIIL_CHECKOUT_DIR=" + AIIL_CHECKOUT_DIR + "\n")
        bashFile.write("export HUSARION_CHECKOUT_DIR=" + husDir + "\n")
        bashFile.write("export PATH=\"$AIIL_CHECKOUT_DIR/bin:$PATH\"\n")
        bashFile.write("\n")
        
        # Append content back
        bashFile.write(content)
    
    # Append additional dynamic elements
    print_subitem("Updating " + rbbBashFile)
    bashFile = open(rbbBashFile, "a+")
    if setupRobot:
        bashFile.write("\n")
        bashFile.write("# SSH-Agent\n")
        bashFile.write("eval \"$(ssh-agent -s)\"\n")
    bashFile.write("\n")
    
    # Append ROS environment variables
    if setupRobot:
        bashFile.write("\n")
        bashFile.write("# ROS Variables\n")
        bashFile.write("export ROS_DOMAIN_ID=" + configRobots[setupRobotName]["ip"] + "\n")
    
    # Append ROS env source
    if setupComp:
        bashFile.write("\n")
        bashFile.write("# Source our ROS workspace\n")
        bashFile.write("source $AIIL_CHECKOUT_DIR/humble_workspace/install/setup.bash\n")
    
    # Done
    bashFile.close()

    # Query if user wishes to automatically source rosbot
    query = False
    if setupRobot:
        query = True
    elif setupComp:
        query = query_yes_no("Configure root ~/.bashrc to automatically source ROSbot workspace?")

    if query:
        # Search for ssh config block
        found = grep(bashrcFile, "ROSBot")

        # If not found, append redbackbots to root bashrc
        if not found:
            print_subitem("Updating ~/.bashrc file")
            bashFile = open(bashrcFile, "a+")
            bashFile.write("\n")
            bashFile.write("# ROSBot Bashrc Source\n")
            if setupComp:
                bashFile.write("source /opt/ros/humble/setup.bash \n")
            bashFile.write("source " + rbbBashFile + "\n")
            bashFile.close()
    else :
        print_warning("Automatic source NOT enabled - will require manual source on every use")


    print_warning(".bashrc configuration has changed." +
                  "source the new bashrc file (using below) and re-run the setup\n" + 
                  "source " + rbbBashFile)
    if setupComp:
        print_warning("HUSARION_CHECKOUT_DIR is set to: '" + HUSARION_CHECKOUT_DIR + "'\n. If this is not correct. Then change before relaunch")
    exit()

def setupBuildRosbot(configRobots, configComputers):
    print_subitem("Configure & Build AIIL ROSBot Humble Workspace")

    # Setup and build go together.
    # If no devel, then run setup version to overlay on husarion workspace
    # If existing devel, then run normal build
    # Both are executed through standalone scripts

    # Check for exiting devel
    humble_workspace = cfg.rosbotHumbleWorkspace()
    develDir = humble_workspace + "/install"
    print_subitem("\t Install Directory: " + develDir)
    command = ""
    if not os.path.exists(develDir):
        # Run initialisation script
        command = cfg.colcon_init_aiil()
    else :
        # Run compilation script
        command = cfg.colcon_make_aiil()
    
    # Execute script
    shell.exec(command, hideOutput=False)

def setupDockerImage(configRobots, configComputers):
    print_status("Creating Docker Image for AIIL Workspace")
    
    dockerDir = cfg.dockerDirectory()
    command = f"docker build -t rmit/aiil_workspace -f {dockerDir}/Dockerfile.humble ."
    print_subitem(f"Executing: {command}")
    shell.exec(command, hideOutput=False)

def setupGit():
    print_status("Checking your Git Configuration")
    gitParams = ["user.name", "user.email", "pull.rebase"]
    for param in gitParams:
        value = shell.capture(["git config " + param], emptyFail=True)
        print("git config " + param + " = " + value)

    query = query_yes_no("Are these details correct?")
    if not query:
        print_status("Updating Git details")
        for param in gitParams:
            query = query_yes_no("Update " + param + "?")
            if query:
                value = query_user("New value for " + param + ": ")
                command = "git config --global " + param + " " + value
                print_status("Executing: " + command)
                shell.exec(command, hideOutput=False)
    else:
        print("Skipping git configuration - details are correct")

def setupHostsAliases(configRobots, configComputers):
    print_status("Configuring /etc/hosts with manage_hosts.py tool")
    print_warning("Manage Hosts program must be run as an administrator. You may ba asked for an admin password")
    manageProgram = cfg.binDirectory() + "/manage_hosts.py"
    commandBase = "sudo " + manageProgram

    if setupRobot:
        command = commandBase + " -r " + setupRobotName
        shell.exec(command, hideOutput=False)
    elif setupComp:
        command = commandBase + " -c " + setupCompName
        shell.exec(command, hideOutput=False)
    elif setupDocker:
        command = commandBase + " -g"
        shell.exec(command, hideOutput=False)

def setupNetworks(configRobots, configWireless):
    print_status("Setting up Networks with set_wireless.py script")
    profile = wireless.getDefaultProfile(configWireless)
    wireless.setProfile(configRobots, configWireless, setupRobotName, profile)
    
def setupRobotLocalFiles(configRobots):
    print_status("Configuring local robot files")
    
    husarionVersion = configRobots[setupRobotName]["husarion"]
    if husarionVersion == "pro2":
        print_subitem("Run Husarion robot setup script")
        command = "sudo /usr/local/sbin/setup_robot_configuration rosbot_2_pro ros2_humble"
        shell.exec(command, hideOutput=False)
    
    print_subitem("Removing local compose/script files")
    rosbotHome = cfg.rosbotHome()
    toRemove = [
        'docker_aiil.sh',
        'compose.yaml',
        'compose-common.yaml',
        'compose.vnc.yaml',
        'flash_firmware.sh',
        'remote_desktop_start.sh',
        'remote_desktop_stop.sh',
        'ros_driver_start.sh',
        'ros_driver_stop.sh',
        'foxglove-rosbot3.json'
    ]
    for file in toRemove:
        absFile = f"{rosbotHome}/{file}"
        print_subitem(f"\t Removing {absFile}")
        if os.path.exists(absFile):
            os.remove(f"{absFile}")
    
    print_subitem("Creating symlinks for files")
    slinkBin = [
        'docker_aiil.sh',
        'flash_firmware.sh',
        # 'remote_desktop_start.sh',
        # 'remote_desktop_stop.sh',
        'ros_driver_start.sh',
        'ros_driver_stop.sh'
    ]
    slinkDocker = [
        'compose.yaml',
        'compose.vnc.yaml'
    ]
    binDir = cfg.binDirectory()
    for file in slinkBin:
        command = f"ln -s {binDir}/rosbot/{husarionVersion}/{file} {rosbotHome}/."
        print_subitem(f"\t {command}")
        shell.exec(command, hideOutput=False)
    
    # Link main compose to version specific compose
    dockerDir = cfg.dockerDirectory()
    composeFile = f"compose-{husarionVersion}.yaml"
    command = f"ln -s {dockerDir}/rosbot/{composeFile} {rosbotHome}/compose.yaml"
    print_subitem(f"\t {command}")
    shell.exec(command, hideOutput=False)
    
    # Copy any additional compose files
    slinkDocker = [
        'compose-common.yaml',
        # 'compose.vnc.yaml'
    ]
    for file in slinkDocker:
        command = f"ln -s {dockerDir}/rosbot/{file} {rosbotHome}/."
        print_subitem(f"\t {command}")
        shell.exec(command, hideOutput=False)

def setupSSHConfig(configRobots, configComputers):
    print_status("Setting up SSH Config")
    sshDir = ssh.sshDirectory
    sshConfig = ssh.sshConfigFile
    print_subitem("SSH Dir: " + sshDir)
    if not os.path.exists(sshDir):
        print_subitem("Making Root (~/.ssh) SSH Directory")
        os.makedirs(sshDir)
        os.chmod(sshDir, 0o700)
    if not os.path.exists(sshConfig):
        print_subitem("Creating Root (~/.ssh) SSH Config File")
        pathlib.Path(sshConfig).touch()
        os.chmod(sshConfig, 0o700)

    # Create the ssh config file in local config folder
    localSSHFile = cfg.sshLocalConfig()
    sshLocalFile = open(localSSHFile, "w+")
    sshLocalFile.write("# Automatically generated ssh config from build_setup.py.\n")
    sshLocalFile.write("# DO NOT MODIFY DIRECTLY\n\n")
    for robot in configRobots.sections():
        print_subitem("\tAdding " + robot)
        setupSSHConfigAdd(sshLocalFile, robot, configRobots[robot]["ip"])
    for comp in configComputers.sections():
        if comp != 'general':
            print_subitem("\tAdding " + comp)
            setupSSHConfigAdd(sshLocalFile, comp, configComputers[comp]["ip"])
    sshLocalFile.close()

    # Copy Local SSH config to destination - and set permissions
    print_subitem("Copying Local SSH to ~/.ssh")
    shutil.copy(localSSHFile, ssh.sshConfigDestination)
    #os.chmod(ssh.sshConfigDestination, 0o600)
    os.chmod(ssh.sshConfigDestination, stat.S_IRUSR | stat.S_IWUSR)

    # Search for ssh config block
    found = grep(sshConfig, "ROSBot")
    
    # If not found, append new block
    robots = cfg.getKeys(configRobots)
    if not found:
        print_subitem("Adding ROSBot robots & computers to SSH Config (~/.ssh/config)")

        # create backup file - as Include must go at the TOP of the .ssh/config
        sshConfigBak = sshConfig + ".bak"
        shutil.copy(sshConfig, sshConfigBak)

        sshFile = open(sshConfig, "w")
        sshFile.write("\n")
        sshFile.write("# ROSBot SSH Config\n")
        sshFile.write("Include " + ssh.sshConfigDestination + "\n")
        sshFile.write("\n")
        sshBak = open(sshConfigBak)
        for line in sshBak:
            sshFile.write(line)
        sshFile.close()

def setupSSHConfigAdd(sshFile, name, ip):
    sshFile.write("Host " + name + "\n")
    sshFile.write("    Hostname " + ssh.getRobotIP(ip) + "\n")
    sshFile.write("    CheckHostIP no\n")
    sshFile.write("    User husarion\n")
    sshFile.write("    StrictHostKeyChecking no\n")
    sshFile.write("\n")

def setupSSHKeys():
    print_status("Setting up SSH Keys")
    pubkeyFile = ssh.pubicKeyFile
    authkeyFile = ssh.authKeyFile()
    print_subitem("pub key file: " + pubkeyFile)
    print_subitem("authorized_keys file: " + authkeyFile)

    # Check SSH Args
    check_command = 'ssh-keygen -l -f ' + pubkeyFile
    retCode = shell.exec(check_command, True)
    if not retCode:
        print_subitem("No id_rsa.pub file - generating")
        print_error("NOT IMPLEMENTED")
    else:
        print_subitem("Using existing id_rsa.pub file")
    
    # Locate pub 
    if retCode:
        checkAuth = ssh.checkAuthKey()
        if not checkAuth:
            print_status("Adding key to authorized keys")
            command = "cat " + pubkeyFile + " >> " + authkeyFile
            shell.exec(command)
        else:
            print_subitem("Key already in authorized keys")

def _main_setup():
    # Check for env configuration, and configure bash
    tmpEnv = cfg.getEnvParameter("AIIL_CHECKOUT_DIR", check=True)
    bashLoaded = tmpEnv != ""

    # Setup bash script
    if not bashLoaded:
        print_warning("bashrc has not been configured. Either reload or configure bash")
        query = query_yes_no("Configure Bash? (Includes ROS environment parameters")
        if query:
            setupBash()
        else:
            print_error("Cannot continue without bashrc being configured")
        exit()
    print()

    # Setup bash script
    if not bashLoaded:
        print_warning("bashrc has not been configured. Either reload or configure bash")
    query = query_yes_no("Configure Bash? (Includes ROS environment parameters")
    if query:
        setupBash()
    elif not bashLoaded:
        print_error("Cannot continue without bashrc being configured")
        exit()
    print()

    # Check for environment variables existing
    AIIL_CHECKOUT_DIR = cfg.getEnvParameter("AIIL_CHECKOUT_DIR")
    HUSARION_CHECKOUT_DIR = cfg.getEnvParameter("HUSARION_CHECKOUT_DIR")

    # Load configs
    config = cfg.loadConfigFile(configDir + "/software.cfg")
    configSoftware = None
    if setupComp:
        configSoftware = config['Software']
    elif setupRobot:
        configSoftware = config['Software.rosbot']
    configROSSoftware = config['Ros']
    configTCSoftware = config['TheConstruct']
    configRobots = cfg.loadConfigFile(configDir + "/robots.cfg")
    configComputers = cfg.loadConfigFile(configDir + "/computers.cfg")
    configWireless = cfg.loadConfigFile(configDir + "/wireless.cfg")
    print_subitem("Configured Robots:")
    [print_subitem("\t* " + robot) for robot in cfg.getKeys(configRobots)]
    print_subitem("Configured Computers:")
    [print_subitem("\t* " + comp) for comp in cfg.getKeys(configComputers)]
    print()
   
    # Check robot/computer exists
    if setupRobot:
        if not configRobots.has_section(setupRobotName):
            print_error("No Configuration parameters for robot: " + setupRobotName)
            exit()
        print_status(f"Configuring for robot: {setupRobotName}")
    if setupComp:
        if not configComputers.has_section(setupCompName):
            print_error("No Configuration parameters for robot: " + setupCompName)
            exit()
        print_status(f"Configuring for computer: {setupCompName}")
    print()

    # Install software
    query = query_yes_no("Install General Software?")
    if query:
        installSoftware(configSoftware)
    print()

    # Ros specific packages - computer only
    if setupComp:
        query = query_yes_no("(Comp Only) Install ROS Specific Additional Packages?")
        if query:
            print_subitem("NOTE: This requires that ROS is already installed on the platform. " +
                        "Follow the instructions here: https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html")
            rosversion='none'
            if setupRobot:
                rosversion=configRobots[setupRobotName]["rosversion"]
            else :
                rosversion=configComputers[setupCompName]["rosversion"]
            installSoftware(configROSSoftware, ros=True, rosversion=rosversion, skipcheck=True)
        print()

        if setupTheConst:
            query = query_yes_no("Install Software for TheConstruct (WARNING USE ON THECONSTRUCT ONLY)?")
            if query:
                installSoftware(configTCSoftware, ros=True, rosversion=rosversion, skipcheck=True)
            print()

    # Snap packages
    configSnap = None
    if setupComp:
        configSnap = config['Snap.computer']
    elif setupRobot:
        husarionVersion = configRobots[setupRobotName]['husarion']
        configSnap = config['Snap.' + husarionVersion]
    query = query_yes_no("Install and/or Configure Pro3 Snap packages for ROS2 Humble")
    if query:
        installSnap(configRobots, configSnap)
    print()

    # Setup Git
    if setupComp:
        query = query_yes_no("(Computer only) Configure Git properties?")
        if query:
            setupGit()
        print()

    # Setup SSH Config
    query = query_yes_no("Setup ~/.ssh/config file?")
    if query:
        setupSSHConfig(configRobots, configComputers)
    print()

    # Setup /etc/hosts Aliases
    query = query_yes_no("Setup /etc/hosts alias?")
    if query:
        setupHostsAliases(configRobots, configComputers)
    print()

    # Setup SSH Keys
    if setupComp:
        query = query_yes_no("(Computer only) Setup SSH Keys?")
        if query:
            setupSSHKeys()
        print()

    # Replace authorised keys (on robot only)
    if setupRobot:
        query = query_yes_no("(Robot only) Replace authorised SSH keys?")
        if query:
            replaceAuthKeys()
        print()

    # Setup ROSbot files and Docker
    # Setup Networks
    if setupRobot:
        query = query_yes_no("(Robot only) Configure local files?")
        if query:
            setupRobotLocalFiles(configRobots)
        print()
        
        query = query_yes_no("(Robot only) Configure Netplan Networks?")
        if query:
            setupNetworks(configRobots, configWireless)
        print()

    # Configure Husarion Git Repositories
    if setupComp:
        query = query_yes_no("Configure & Build Repositories?")
        if query:
            if False:
                setupBuildHusarion(configRobots, configComputers, config)
            else:
                print_subitem("Husarion software build not required - disabled")
            setupBuildRosbot(configRobots, configComputers)
        print()

def _main_setupDocker():
    print_status("Setup Docker environment")

    # Check for environment variables existing, otherwise set manually
    tmpEnv = cfg.getEnvParameter("AIIL_CHECKOUT_DIR", check=True)
    if tmpEnv != "":
        AIIL_CHECKOUT_DIR = tmpEnv
    
    tmpEnv = cfg.getEnvParameter("HUSARION_CHECKOUT_DIR", check=True)
    if tmpEnv != "":
        HUSARION_CHECKOUT_DIR = tmpEnv

    # Load configs
    configRobots = cfg.loadConfigFile(configDir + "/robots.cfg")
    configComputers = cfg.loadConfigFile(configDir + "/computers.cfg")

    # Run docker build
    query = query_yes_no("Execute Docker build")
    if query:
        setupDockerImage(configRobots, configComputers)
    print()
    

# Main entry point
if __name__ == "__main__":
    print_status("Commencing Build Setup")
    print_warning("If you are unsure about any answer, always select yes (Y)")
   
    # Process args
    parser = argparse.ArgumentParser(description='Setup the build for a robot/computer')
    parser.add_argument('-r', dest="robot", type=str, help='Robot to setup Build for (conflicts with "computer")')
    parser.add_argument('-c', dest="computer", type=str, help='Computer to setup Build for (conflicts with "robot")')
    parser.add_argument('-g', dest="general", action='store_true', help='Configure a General Computer/Device')
    parser.add_argument('-t', dest="theconstruct", action='store_true', help='Configure TheConstruct (in addition to a general computer)')
    parser.add_argument('-d', dest="docker", action='store_true', help='Configure a Docker environment')
    args = parser.parse_args()
    setupCount = 0
    if (args.robot is not None):
        setupRobot = True
        setupRobotName = args.robot
        setupCount += 1
    if (args.computer is not None):
        setupComp = True
        setupCompName = args.computer
        setupCount += 1
    if args.general:
        setupComp = True
        setupCompName = 'general'
        setupCount += 1
    if args.theconstruct:
        setupTheConst = True
    if args.docker:
        setupDocker = True
        setupCount += 1
    if (not setupRobot) and (not setupComp) and (not setupDocker):
        print_error("No selection of Robot, Computer or Docker")
        parser.print_help()
        exit()
    if setupCount > 1:
        print_error("Cannot setup for multiple forms, choose only one")
        parser.print_help()
        exit()

    if setupRobot:
        print_status("Running Setup for ROBOT: " + setupRobotName)
    elif setupComp:
        print_status("Running Setup for COMPUTER " + setupCompName)


    # Loading paths
    print_subitem("AIIL_CHECKOUT_DIR = " + AIIL_CHECKOUT_DIR)
    if setupComp:
        print_subitem("HUSARION_CHECKOUT_DIR = " + HUSARION_CHECKOUT_DIR)
    print_subitem("Bin Directory = " + binDir)
    print_subitem("Config Directory = " + configDir)
    print()

    # Use setup based on robot/comp or docker
    if setupComp:
        _main_setup()
    elif setupRobot:
        _main_setup()
        _main_setupDocker()
    elif setupDocker:
        _main_setupDocker()

    print_status("Build Setup Complete")

