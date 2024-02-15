#! /usr/bin/env python3

'''
get_robot_info:
- Retrieves configuration information of a robot from config

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
AIIL_CHECKOUT_DIR = os.path.abspath(binDir + "/../")

# Global tracking vars
robotName = None
quiet = False

# Main entry point
if __name__ == "__main__": 
    # Process args
    parser = argparse.ArgumentParser(description='Get configuration infomration of a robot')
    parser.add_argument('-r', dest="robot", type=str, help='Robot of which to get the configuration')
    parser.add_argument('-i', dest="ip", action='store_true', help='Get the robot ip')
    parser.add_argument('-q', dest="quiet", action='store_true', help='Quiet: Only output the desired information without logging')
    args = parser.parse_args()
    if (args.robot is not None):
        robotName = args.robot
    quiet = args.quiet

    if robotName is None:
        if not quiet:
            print_error("No robot name provided")
        exit()
    if not quiet: print_status(f"Getting config for {robotName}")

    # Load environment
    AIIL_CHECKOUT_DIR = cfg.getEnvParameter("AIIL_CHECKOUT_DIR")

    # Load configs
    configDir = cfg.configDirectory()
    configRobots = cfg.loadConfigFile(configDir + "/robots.cfg")
    
    # Process IP
    if (args.ip):
        ip = configRobots[robotName]["ip"]
        if quiet:
            print(ip, end=""    )
        else:
            print_subitem(f"ip = {ip}")
        
