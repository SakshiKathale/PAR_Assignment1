
#! /usr/bin/env python3

'''
Install snap packages using Python subprocess
'''

import utils.echo as echo
import utils.shellEscape as shell

def configure_snap_property(package, property, value):
    echo.print_subitem("Snap: Configuring {pkg_name} property {prop_name} to {prop_value}".format(pkg_name=package, prop_name=property, prop_value=value))
    command = "sudo snap set " + package + " " + property + "=" + value
    shell.exec(command, hideOutput=False)

def install_snap_packages(package, channel=None):
    # Check if the package is already installed
    command = "snap list " + package
    if shell.check_output(command):
        echo.print_subitem("Snap: {pkg_name} is already installed".format(pkg_name=package))
    else:
        echo.print_subitem("Snap: Installing {pkg_name} (channel {channel})".format(pkg_name=package, channel=channel))
        command = "sudo snap install " + package
        if channel != None:
            command = command + f" --channel={channel}"
        shell.exec(command, hideOutput=False)

def remove_snap_package(package):
    # Check if the package is already installed
    command = "snap list " + package
    if shell.check_output(command):
        echo.print_subitem("Snap: Removing {pkg_name}".format(pkg_name=package))
        command = "sudo snap remove " + package
        shell.exec(command, hideOutput=False)
    else:
        echo.print_subitem("Snap: {pkg_name} is not installed".format(pkg_name=package))

