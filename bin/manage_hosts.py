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
import sys

from utils.echo import (
    print_error,
    print_status,
    print_warning,
    print_subitem,
)
from utils.grep import (
    grep
)
import utils.shellEscape as shell


# Global Parameters
hostsFile       = "/etc/hosts"
hostsFileBak    = hostsFile + ".bak"

def appendNew(alias, ip):

    # Search for existing block of ROSbot elements
    strKey = "ROSBot"
    existing = grep(hostsFile, strKey)

    if existing:
        # Iterate through backup file and insert new line
        fhBak = open(hostsFileBak)
        fhHosts = open(hostsFile, "w")
        added = False
        for line in fhBak:
            # Repeat existing content
            fhHosts.write(line)

            if (not added) and re.search(strKey, line):
                # Found block - append new line
                fhHosts.write(aliasLine(alias, ip) + "\n")
                added = True

    else :
        # Append to end
        fhHosts = open(hostsFile, "a+")
        fhHosts.write("\n")
        fhHosts.write("# ROSBot hosts\n")
        fhHosts.write(aliasLine(alias, ip) + "\n")
        fhHosts.write("\n")

    return

def replaceExisting(alias, ip):
    # Iterate through backup file and insert new line
    fhBak = open(hostsFileBak)
    fhHosts = open(hostsFile, "w")
    added = False
    for line in fhBak:
        if (not added) and re.search(alias, line):
            # Found replace entry
            fhHosts.write(aliasLine(alias, ip) + "\n")
            added = True
        else :
            # Repeat existing content
            fhHosts.write(line)
    return


def aliasLine(alias, ip):
    return ip + "    " + alias

if __name__ == "__main__":
    print_status("Manage /etc/hosts for ROSBot")
    print_warning("To function, this program must be run with administrator privilages")
    print()

    # Parse args for managing the host
    parser = argparse.ArgumentParser(description='Manage /etc/hosts for ROSBot library. Updates the given alias to the given ip in the /etc/hosts file. A backup of the file is made in /etc/hosts.bak')
    parser.add_argument('alias', type=str, help='Alias of the IP host')
    parser.add_argument('ip', type=str, help='IP address for the alias')
    args = parser.parse_args()
    alias = args.alias
    ip = args.ip
    print_status("Parameters")
    print_subitem("alias: " + alias)
    print_subitem("ip: " + ip)

    # Check ordering of alias/ip
    if not re.search("^[0-9]+[.][0-9]+[.][0-9]+[.][0-9]+$", ip):
        print_error("IP is the wrong format, no IP address: " + ip)
        exit()

    # Create Hosts backup
    print_subitem("Creating backup")
    print_warning("If something goes wrong, replace /etc/hosts with the backup or with the below information. CURRENT STATE OF /etc/hosts")
    print()
    shell.exec("cat " + hostsFile, hideOutput = False)
    shutil.copy(hostsFile, hostsFileBak)

    # Search for existing alias
    exists = grep(hostsFile, alias)
    print_status(exists)

    # If present, modify in-place
    # If not present, append new alias to ROSBot configuration location
    if exists:
        print_status("Modfyig existing alias")
        replaceExisting(alias, ip)
    else :
        print_status("Adding new alias")
        appendNew(alias, ip)

    exit()

