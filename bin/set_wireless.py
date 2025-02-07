#!/usr/bin/env python3

import argparse
import os
import shutil
import filecmp

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
import utils.sshscp as ssh
import utils.wireless as wireless

# Global Parameters
fullSetupPath = os.path.abspath(__file__)
binDir = os.path.dirname(fullSetupPath)
configDir = os.path.abspath(binDir + "/../config/")
AIIL_CHECKOUT_DIR = os.path.abspath(binDir + "/../")

# Global setup type parameters
robotName  = None
testMode   = False

def selectProfile(configWireless):
    print_status("Select Wireless Profile")
    print_subitem("Available Profiles:")
    
    profile = None
    for index, option in enumerate(configWireless, start=1):
        print_subitem(f"{index}. Profile: {option}")
    
    selected = input("Choose profile: ")
    
    try:
        selected_index = int(selected)
        # Make default profile, first in the config
        if selected_index == 1:
            selected_index = 2
            
        if selected_index > 1 and selected_index <= len(configWireless):
            profile = list(configWireless.keys())[selected_index - 1]
        else:
            print_error("Invalid profile number")
    except ValueError:
        print_error("Invalid input. Please enter a number.")
    print()
    
    return profile

if __name__ == "__main__":
    parser = argparse.ArgumentParser('Choose wireless profile')
    parser.add_argument('-r', dest="robot", type=str, required=True, help='Robot (required) to set correct IP')
    parser.add_argument('-p', dest="profile", type=str, help='Profile to use. If not provided, available profiles are listed')
    parser.add_argument('-d', dest="default", action="store_true", help='Use default profile')
    parser.add_argument('-t', dest="test", action="store_true", help='Enable test mode - does not replace netplan config')
    args = parser.parse_args()
    
    # Check for env configuration, and configure bash
    tmpEnv = cfg.getEnvParameter("AIIL_CHECKOUT_DIR", check=True)
    bashLoaded = tmpEnv != ""

    # Setup bash script
    if not bashLoaded:
        print_error("Cannot continue without bashrc being configured")
        exit()

    # Check for environment variables existing
    AIIL_CHECKOUT_DIR = cfg.getEnvParameter("AIIL_CHECKOUT_DIR")
    HUSARION_CHECKOUT_DIR = cfg.getEnvParameter("HUSARION_CHECKOUT_DIR")

    # Load configs
    configRobots = cfg.loadConfigFile(configDir + "/robots.cfg")
    configWireless = cfg.loadConfigFile(configDir + "/wireless.cfg")
  
    # Check robot exists
    if args.robot not in configRobots:
        print_error(f"Robot '{args.robot}' does not exist")
        exit()
    robotName = args.robot
  
    # Check profile exists
    if args.profile:
        profile = args.profile
    elif args.default:
        print_subitem("Using default profile")
        profile = wireless.getDefaultProfile(configWireless)
    else:
        profile = selectProfile(configWireless)
    
    if profile not in configWireless:
        print_error(f"Profile '{profile}' does not exist")
        exit()
        
    # Set test mode
    if args.test:
        print_warning("Test mode enabled")
        testMode = True

    # Set the profile
    wireless.setProfile(configRobots, configWireless, robotName, profile, testMode=testMode)
