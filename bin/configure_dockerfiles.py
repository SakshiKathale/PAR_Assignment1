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
)

# Global Parameters
fullSetupPath = os.path.abspath(__file__)
binDir = os.path.dirname(fullSetupPath)
configDir = os.path.abspath(binDir + "/../config/")
AIIL_CHECKOUT_DIR = os.path.abspath(binDir + "/../")
HUSARION_CHECKOUT_DIR = os.path.abspath(AIIL_CHECKOUT_DIR + "/../" + cfg.husarion_workspace)

# Global vars
rosversion = 'noetic'

def aptCommands(configSoftware):
    configGeneralSoftware = configSoftware['Software']
    configROSSoftware = configSoftware['Ros']

    # General Software
    packageList = []
    for pkg in configGeneralSoftware:
        if configGeneralSoftware.getboolean(pkg):
            packageList.append(pkg)
    print_status("General package list")
    outputAptCommand(packageList)
    print()

    # ROS Software
    packageList = []
    for pkg in configROSSoftware:
        if configROSSoftware.getboolean(pkg):
            pkg = "ros-" + rosversion + "-" + pkg
            packageList.append(pkg)
    print_status("ROS package list")
    outputAptCommand(packageList)
    print()

def outputAptCommand(packageList):
    fullStr = " \\\n        ".join(packageList)
    print('''RUN apt update && \ 
    apt install -y \\''')
    print("        "  + fullStr)

# Main entry point
if __name__ == "__main__":
    print_status("Commencing Configuration")
   
    # Process args
    parser = argparse.ArgumentParser(description='Configure files for Docker')
    parser.add_argument('-r', dest="rosversion", required=True, type=str, help='ROS Version to use')
    args = parser.parse_args()

    # Loading paths
    print_subitem("AIIL_CHECKOUT_DIR = " + AIIL_CHECKOUT_DIR)
    print_subitem("HUSARION_CHECKOUT_DIR = " + HUSARION_CHECKOUT_DIR)
    print_subitem("Bin Directory = " + binDir)
    print_subitem("Config Directory = " + configDir)
    print()

    # Check for environment variables existing
    tmpEnv = cfg.getEnvParameter("AIIL_CHECKOUT_DIR", check=True)
    if tmpEnv != "":
        AIIL_CHECKOUT_DIR = tmpEnv
    
    tmpEnv = cfg.getEnvParameter("HUSARION_CHECKOUT_DIR", check=True)
    if tmpEnv != "":
        HUSARION_CHECKOUT_DIR = tmpEnv
    
    # Set globalvars
    rosversion = args.rosversion

    # Load configs
    configSoftware = cfg.loadConfigFile(configDir + "/software.cfg")
    configRobots = cfg.loadConfigFile(configDir + "/robots.cfg")
    configComputers = cfg.loadConfigFile(configDir + "/computers.cfg")
    print()
   
    # Generate commands
    aptCommands(configSoftware)
    print()

    print_status("Complete")

