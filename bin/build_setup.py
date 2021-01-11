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
import subprocess
import sys

import utils.config as cfg
from utils.echo import (
    print_error,
    print_status,
    print_warning,
    print_progress,
    print_subitem,
    query_yes_no
)
from utils.grep import (
    grep
)
import utils.shellEscape as shell
import utils.sshscp as ssh

# Global Parameters
fullSetupPath = os.path.abspath(__file__)
binDir = os.path.dirname(fullSetupPath)
configDir = os.path.abspath(binDir + "/../config/")
ROSBOT_CHECKOUT_DIR = os.path.abspath(binDir + "/../")
HUSARION_CHECKOUT_DIR = os.path.abspath(ROSBOT_CHECKOUT_DIR + "/../" + cfg.husarion_workspace)

# Global setup type parameters
setupRobot      = False
setupRobotName  = None
setupComp       = False
setupCompName   = None

def installSoftware(software, ros=False, rosversion='none'):
    # Load this here to avoid script delays
    print_status("Installing")
    from utils.aptinstall import aptinstall

    # Load Software to install
    counter = 0
    for pkg in software:
        progress = round(counter / len(software) * 100)

        # If ROS package - prepend with ros package name
        if ros:
            pkg = "ros-" + rosversion + "-" + pkg

        if software.getboolean(pkg):
            print_progress("Installing " + pkg, progress)
            installed = aptinstall(pkg)
            #installed = False;
            if not installed:
                #print_warning("INSTALLATION TEMPORARILY DISABLED DURING SCRIPT TESTING FOR SAFETY")
                print_error("Package installation failed, terminating")
                exit()
        else:
            print_progress("Skipping " + pkg, progress)
        counter += 1
    print_progress("Installation done", 100)


def replaceAuthKeys():
    localAuthFile = ssh.authKeyFile()
    target = ssh.sshDirectory
    shutil.copy(localAuthFile, target)


def setupBash():
    print_status("Configuring Bash Environment")
    rbbBashFile = os.path.abspath(ROSBOT_CHECKOUT_DIR + "/.bashrc")
    bashrcFile = os.path.expanduser("~/.bashrc")
    
    # Create redbackbots source file
    templateBashrc = os.path.abspath(ROSBOT_CHECKOUT_DIR + "/config/bashrc")
    shutil.copy(templateBashrc, rbbBashFile)
    print_subitem("Copying " + templateBashrc + " to " + rbbBashFile)

    # Set Husarion workspace based on device type
    if setupRobot:
        HUSARION_CHECKOUT_DIR = os.path.expanduser("~/" + cfg.husarion_workspace)

    # Append additional dynamic elements
    print_subitem("Updating " + rbbBashFile)
    bashFile = open(rbbBashFile, "a+")
    bashFile.write("\n")
    bashFile.write("# ROSBot Environment Settings\n")
    bashFile.write("export ROSBOT_CHECKOUT_DIR="+ROSBOT_CHECKOUT_DIR+"\n")
    bashFile.write("export HUSARION_CHECKOUT_DIR="+HUSARION_CHECKOUT_DIR+"\n")
    bashFile.write("export PATH=\"$ROSBOT_CHECKOUT_DIR/bin:$PATH\"\n")
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
            bashFile.write("source " + rbbBashFile + "\n")
            bashFile.close()
    else :
        print_warning("Automatic source NOT enabled - will require manual source on every use")


    print_warning(".bashrc configuration has changed." +
                  "source the new bashrc file (using below) and re-run the setup\n" + 
                  "source " + rbbBashFile)
    print_warning("HUSARION_CHECKOUT_DIR is set to: '" + HUSARION_CHECKOUT_DIR + "'\n. If this is not correct. Then change before relaunch")
    exit()

def setupBuildHusarion(configRobots, configComputers):
    # Create and configure CMake
    print_status("Configure & Build Husarion Workspace")

    # Setup Husarion Repos
    setupHusarionRepos(configRobots, configComputers)

    # Execute standalone catkin_make script for husarion_ws
    print_status("Building Husarion Workspace")
    binDir = cfg.binDirectory()
    script = binDir + "/catkin/catkin_make_husarion"
    shell.exec(script, hideOutput=False)

def setupBuildRosbot(configRobots, configComputers):
    print_subitem("Configure & Build AIIL ROSBot Melodic Workspace")

    # Setup and build go together.
    # If no devel, then run setup version to overlay on husarion workspace
    # If existing devel, then run normal build
    # Both are executed through standalone scripts

    # Check for exiting devel
    melodic_workspace = cfg.rosbotMelodicWorkspace()
    develDir = melodic_workspace + "/devel"
    command = ""
    if not os.path.exists(develDir):
        # Run initialisation script
        command = cfg.catkin_init_aiil()
    else :
        # Run compilation script
        command = cfg.catkin_make_aiil()
    
    # Execute script
    shell.exec(command, hideOutput=False)

def setupGit():
    print_status("Checking your Git Configuration")
    print("If the user info is incorrect, please configure it like:")
    print("\tgit config user.name Tim")
    print("\tgit config user.email s123456@student.rmit.edu.au")
    
    gitUser = shell.capture(["git config user.name"])
    gitEmail = shell.capture(["git config user.email"])
    print("Your user name:", gitUser)
    print("Your email:", gitEmail)


def setupHusarionRepos(configRobots, configComputers):
    print_status("Setting up Husarion ROS Repositories")

    # Repos Config
    configRepos = cfg.loadConfigFile(configDir + "/repos.cfg")

    # Check for Husarion Repository
    develDir = HUSARION_CHECKOUT_DIR + "/devel"
    if not os.path.exists(develDir):
        # Initialise Husarion Directory
        rosversion = 'melodic'
        if setupRobot:
            rosversion = configRobots[setupRobotName]['rosversion']
        elif setupComp:
            rosversion = configComputers[setupCompName]['rosversion']
        
        # Ensure directory exists
        if not os.path.exists(HUSARION_CHECKOUT_DIR):
            os.makedirs(HUSARION_CHECKOUT_DIR)

        # Run initialisation script
        command = cfg.catkin_init_husarion() + " " + rosversion
        shell.exec(command, hideOutput=False)

    # Iterate through each repo
    for repo in configRepos.sections():
        if configRepos[repo]['type'] == cfg.husarion_workspace:
            print_subitem("Repo: " + repo)

            # Check if exists
            wsDir = HUSARION_CHECKOUT_DIR + "/src/" + repo
            dirExists = os.path.exists(wsDir)

            # Check if git version
            gitDir = wsDir + "/.git"
            gitExists = os.path.exists(gitDir)
            print_subitem("\twsDir: " + wsDir)
            print_subitem("\tgitDir: " + gitDir)

            # Clone/Update
            if gitExists:
                # Update
                print_subitem("Updating Repo: " + repo)
                os.chdir(wsDir)
                command = "git pull"
                shell.exec(command, hideOutput=False)
                os.chdir(ROSBOT_CHECKOUT_DIR)

            else :
                print_subitem("Cloning Repo: " + repo)
                if dirExists:
                    print_subitem("\tMoving old repo out and replacing with cloned repo")
                    # Move out-of-way
                    saveDir = HUSARION_CHECKOUT_DIR + "/src/" + "orig_image/."
                    if not os.path.exists(saveDir):
                        os.makedirs(saveDir)
                    shutil.move(wsDir, saveDir)
                    # Ensure catkin_ignore set
                    shell.exec("touch " + saveDir + "/CATKIN_IGNORE")

                # Clone
                os.chdir(HUSARION_CHECKOUT_DIR + "/src")
                command = "git clone " + configRepos[repo]['giturl'] + " " + repo
                shell.exec(command, hideOutput=False)
                os.chdir(ROSBOT_CHECKOUT_DIR)

            

def setupHostsAliases(configRobots, configComputers):
    print_status("Configuring /etc/hosts with manage_hosts.py tool")
    print_warning("Manage Hosts program must be run as an administrator. You may ba asked for an admin password")
    manageProgram = cfg.binDirectory() + "/manage_hosts.py"
    commandBase = "sudo " + manageProgram

    for robot in configRobots.sections():
        if robot != setupRobotName:
            print_subitem("Updating " + robot)
            command = commandBase + " " + robot + " " + ssh.getRobotIP(configRobots[robot]["ip"])
            shell.exec(command, hideOutput=True)

    for comp in configComputers.sections():
        if comp != setupCompName:
            print_subitem("Updating " + comp)
            command = commandBase + " " + comp + " " + ssh.getRobotIP(configComputers[comp]["ip"])
            shell.exec(command, hideOutput=True)


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
        os.chmod(sshConfig, 0o600)

    # Create the ssh config file in local config folder
    localSSHFile = cfg.sshLocalConfig()
    sshLocalFile = open(localSSHFile, "w+")
    sshLocalFile.write("# Automatically generated ssh config from build_setup.py.\n")
    sshLocalFile.write("# DO NOT MODIFY DIRECTLY\n\n")
    for robot in configRobots.sections():
        print_subitem("\tAdding " + robot)
        setupSSHConfigAdd(sshLocalFile, robot, configRobots[robot]["ip"])
    for comp in configComputers.sections():
        print_subitem("\tAdding " + comp)
        setupSSHConfigAdd(sshLocalFile, comp, configComputers[comp]["ip"])
    sshLocalFile.close()

    # Copy Local SSH config to destination
    print_subitem("Copying Local SSH to ~/.ssh")
    shutil.copy(localSSHFile, ssh.sshConfigDestination)

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
    else:
        print_warning("SSH Config already configured. Updating this is not yet implemented")

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
    authkeyFile = ssh.authKeyFile
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

    

# Main entry point
if __name__ == "__main__":
    print_status("Commencing Build Setup")
    print_warning("If you are unsure about any answer, always select yes (Y)")
   
    # Process args
    parser = argparse.ArgumentParser(description='Setup the build for a robot/computer')
    parser.add_argument('-r', dest="robot", type=str, help='Robot to setup Build for (conflicts with "computer")')
    parser.add_argument('-c', dest="computer", type=str, help='Computer to setup Build for (conflicts with "robot")')
    args = parser.parse_args()
    if (args.robot is not None):
        setupRobot = True
        setupRobotName = args.robot
    if (args.computer is not None):
        setupComp = True
        setupCompName = args.computer
    if (not setupRobot) and (not setupComp):
        print_error("No selection of Robot or Computer")
        parser.print_help()
        exit()
    if (setupRobot) and (setupComp):
        print_error("Cannot setup for both robot and computer")
        parser.print_help()
        exit()

    if setupRobot:
        print_status("Running Setup for ROBOT: " + setupRobotName)
    elif setupComp:
        print_status("Running Setup for COMPUTER " + setupCompName)


    # Loading paths
    print_subitem("ROSBOT_CHECKOUT_DIR = " + ROSBOT_CHECKOUT_DIR)
    print_subitem("HUSARION_CHECKOUT_DIR = " + HUSARION_CHECKOUT_DIR)
    print_subitem("Bin Directory = " + binDir)
    print_subitem("Config Directory = " + configDir)
    print()
   
    # Check for env configuration, and configure bash
    tmpEnv = cfg.getEnvParameter("ROSBOT_CHECKOUT_DIR", check=True)
    bashLoaded = tmpEnv != ""

    if not bashLoaded:
        print_warning("bashrc has not been configured. Configuring bash")
        setupBash()
        exit()

    # Setup bash script
    query = query_yes_no("Configure Bash?")
    if query:
        setupBash()
        exit()
    print()

    # Check for environment variables existing
    ROSBOT_CHECKOUT_DIR = cfg.getEnvParameter("ROSBOT_CHECKOUT_DIR")
    HUSARION_CHECKOUT_DIR = cfg.getEnvParameter("HUSARION_CHECKOUT_DIR")

    # Load configs
    config = cfg.loadConfigFile(configDir + "/software.cfg")
    configSoftware = config['Software']
    configROSSoftware = config['Ros']
    configRobots = cfg.loadConfigFile(configDir + "/robots.cfg")
    configComputers = cfg.loadConfigFile(configDir + "/computers.cfg")
    print_subitem("Configured Robots:")
    [print_subitem("\t* " + robot) for robot in cfg.getKeys(configRobots)]
    print_subitem("Configured Computers:")
    [print_subitem("\t* " + comp) for comp in cfg.getKeys(configComputers)]
    print()
   
    # Check robot/computer exists
    if setupRobot:
        if not configRobots.has_section(setupRobotName):
            print_error("No Configuruation parmaeters for robot: " + setupRobotName)
            exit()
    if setupComp:
        if not configComputers.has_section(setupCompName):
            print_error("No Configuruation parmaeters for robot: " + setupCompName)
            exit()

    # Install software
    query = query_yes_no("Install General Software?")
    if query:
        installSoftware(configSoftware)
    print()

    query = query_yes_no("Install ROS Specific Additional Packages?")
    if query:
        rosversion='none'
        if setupRobot:
            rosversion=configRobots[setupRobotName]["rosversion"]
        else :
            rosversion=configComputers[setupCompName]["rosversion"]
        installSoftware(configROSSoftware, ros=True, rosversion=rosversion)
    print()

    # Setup Git
    if setupComp:
        query = query_yes_no("(Computer only) Setup Git?")
        if query:
            setupGit()
        print()

    # Setup SSH Config
    query = query_yes_no("Setup SSH Config?")
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

    # Repalce authorised keys (on robot only)
    if setupRobot:
        query = query_yes_no("(Robot only) Replace authorised SSH keys?")
        if query:
            replaceAuthKeys()
        print()

    # Configure Husarion Git Repositories
    query = query_yes_no("Configure & Build Repositories?")
    if query:
        setupBuildHusarion(configRobots, configComputers)
        setupBuildRosbot(configRobots, configComputers)
    print()

    print_status("Build Setup Complete")

