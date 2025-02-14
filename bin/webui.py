#!/usr/bin/env python3

import utils.config as cfg
from utils.echo import (
    print_error,
    print_status,
    print_warning,
    print_progress,
    print_subitem,
    query_yes_no
)
import utils.shellEscape as shell
import utils.snapinstall as snap
import utils.sshscp as scp

import argparse
import os
import shutil

# Global Parameters
fullSetupPath = os.path.abspath(__file__)
binDir = os.path.dirname(fullSetupPath)
configDir = os.path.abspath(binDir + "/../config/")
AIIL_CHECKOUT_DIR = os.path.abspath(binDir + "/../")
HUSARION_CHECKOUT_DIR = os.path.abspath(AIIL_CHECKOUT_DIR + "/humble_workspace/src/" + cfg.husarion_workspace)

def copy_json_files():
    print_status("Copying JSON files to Foxglove directory")
    src_dir = os.path.abspath(AIIL_CHECKOUT_DIR + "/humble_workspace/src/aiil_rosbot_demo/rviz")
    dest = cfg.foxgloveLayoutDir()
    print_subitem(f"Foxglove Directory: {dest}")
    print_subitem("sudo password may be required")
    
    for file in os.listdir(src_dir):
        if file.endswith(".json"):
            print_subitem(f"Copying: {file}")
            src = os.path.join(src_dir, file)
            command = f"sudo cp {src} {dest}"
            shell.exec(command, hideOutput=False)

def set_robot_ip(robotName):
    print_status(f"Setting Foxglove WebUI to connect to robot: {robotName}")
    configRobots = cfg.loadConfigFile(configDir + "/robots.cfg")
    
    # Check Robot Name is valid
    ip = None
    if robotName == 'default':
        ip = "0"
    elif robotName:
        if not configRobots.has_section(robotName):
            print_error("No Configuration parameters for robot: " + robotName)
            exit()
        ip = configRobots[robotName]["ip"]
    
    # Get Robot IP
    print_subitem(f"Using IP: {ip}")
    print_subitem(f"Changing snap properties may require sudo password")
    
    # Call snap set
    snap.configure_snap_property("husarion-webui", "ros.domain-id", ip)

def start_webui():
    print_status("Starting Foxglove WebUI...")
    print_subitem("sudo password may be required")
    command = "sudo husarion-webui.start"
    shell.exec(command, hideOutput=False)

def stop_webui():
    print_status("Stopping Foxglove WebUI...")
    print_subitem("sudo password may be required")
    command = "sudo husarion-webui.stop"
    shell.exec(command, hideOutput=False)

def getLayoutFile():
    layout_file = None
    
    ros2_dir = cfg.foxgloveLayoutDir()
    files = [file for file in os.listdir(ros2_dir) if file.endswith(".json")]
    files = [file.replace(".json", "") for file in files]
    files = [file.replace("foxglove-", "") for file in files]    
    files.sort()
    files.append("ABORT - use existing layout")
    
    print_status("Available compose files:")
    for i, file in enumerate(files):
        print_subitem(f"{i+1}. {file}")
        
    chosen = False
    while not chosen:
        try:
            raw_choice = input("Choose a number: ")
            choice = int(raw_choice)
            if choice < 1 or choice > len(files):
                print_warning("Invalid choice. Please choose a number from the list.")
            elif choice == len(files):
                print_status("Leaving current compose file in place")
                layout_file = None
                chosen = True
            else:
                layout_file = files[choice-1]
                chosen = True
        except ValueError:
            if raw_choice == "":
                print_status("Leaving current compose file in place")
                layout_file = None
                chosen = True
            else:
                print_warning("Invalid choice. Please enter a number.")
            
    return layout_file

def choose_layout():
    print_status("Choosing Foxglove WebUI layout")
    
    # Choose layout
    layout_file = getLayoutFile()
    print()
    
    if layout_file is not None:
        print_subitem(f"Using layout {layout_file}")
        command = f"sudo snap set husarion-webui webui.layout={layout_file}"
        print_subitem("sudo password may be required")
        shell.exec(command, hideOutput=False)

def main():
    parser = argparse.ArgumentParser("Configure and use Foxglove WebUI with Husarion")
    parser.add_argument("--copy-json-files", "-c", action="store_true", help="Copy JSON foxglove files to the correct directory")
    parser.add_argument("--robot", "-r", type=str, help="Robot to set Foxglove to connect to, or 'default' to use no domain id")
    parser.add_argument("--start", "-s", action="store_true", help="Start Husarion WebUI")
    parser.add_argument("--stop", "-t", action="store_true", help="Stop Husarion WebUI")
    parser.add_argument("--layout", "-l", action="store_true", help="Choose layout from the provided list")
    args = parser.parse_args()

    print_status("Foxglove WebUI Control")
    print()
    
    if args.copy_json_files:
        copy_json_files()
    elif args.robot:
        set_robot_ip(args.robot)
    elif args.start:
        start_webui()
    elif args.stop:
        stop_webui()
    elif args.layout:
        choose_layout()
    else:
        print_error("Invalid parameter. See '-h' for options.")

if __name__ == "__main__":
    main()
