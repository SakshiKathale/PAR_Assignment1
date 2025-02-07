#! /usr/bin/env python3

'''
Wrapper for configuring wireless, used by multiple scritps
'''

import utils.config as cfg
from utils.echo import (
    print_error,
    print_status,
    print_warning,
    print_subitem
)
import utils.shellEscape as shell

'''
Return the default wireless profile
'''
def getDefaultProfile(configWireless):
    return list(configWireless.keys())[1]

'''
Set the wireless profile in netplan
'''
def setProfile(configRobots, configWireless, robotName, profile, testMode=False):
    print_status(f"Setting up Netplan Networks with profile: {profile}")
    npFileBase = "01-network-manager-all.yaml"
    npFile = cfg.configDirectory() + "/rosbot/" + npFileBase
    
    ip = configRobots[robotName]["ip"]
    ssid = configWireless[profile]["ssid"]
    pwd = configWireless[profile]["pwd"]
    nameservers = configWireless[profile]["nameservers"]
    
    # Read the contents of the netplan template
    with open(npFile, 'r') as file:
        file_content = file.read()

    # Replace strings
    file_content = file_content.replace('ROBOT_IP', ip)
    file_content = file_content.replace('SSID', ssid)
    file_content = file_content.replace('PWD', pwd)
    file_content = file_content.replace('NAMESERVERS', nameservers)
    
    # Show new file
    print_subitem("New netplan file.\n" + file_content)
    
    # Write temporary netplan file
    print_subitem("Updating netplan file - requires sudo")
    tmpFile = "/tmp/" + npFileBase
    if not testMode:
        with open(tmpFile, 'w') as file:
            file.write(file_content)
    
    if not testMode:
        # Copy file into place (with sudo)
        npFolder = "/etc/netplan"
        npOutFile = f"{npFolder}/{npFileBase}"
        command = f"sudo cp {tmpFile} {npOutFile}"
        shell.exec(command, hideOutput=False)
    
        # Cleanup
        command = f"sudo rm -f {tmpFile}"
        shell.exec(command, hideOutput=False)
        
        # Update netplan
        print_subitem("Updating Netplan - requires sudo")
        print_warning("This will reset the network, remote terminals may be disconnected")
        command = f"sudo netplan -d apply"
        shell.exec(command, hideOutput=False)
    else:
        print_warning("Test mode enabled - netplan not updated")
