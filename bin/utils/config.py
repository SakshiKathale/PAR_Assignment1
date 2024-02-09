#! /usr/bin/env python3

'''
Quick wrappers around loading of RBB configuration including
- Environment settings
- INI Config files
'''

import configparser
import os

from utils.echo import (
    print_error,
    print_warning
)

'''
Static names/variables
'''
husarion_workspace = 'husarion_ws'
rosbot_workspace = 'aiil_workspace'

'''
Location of bin directory
'''
def binDirectory():
    return checkoutDirectory() + "/bin"

'''
Catkin script for initialising husarion
'''
def catkin_init_husarion():
    return binDirectory() + "/catkin/catkin_init_husarion"

'''
Catkin script for initialising AIIL ROSBot
'''
def colcon_init_aiil():
    return binDirectory() + "/catkin/colcon_init_aiil"

'''
Catkin script for building husarion
'''
def catkin_make_husarion():
    return binDirectory() + "/catkin/catkin_make_husarion"

'''
Catkin script for building AIIL ROSBot
'''
def colcon_make_aiil():
    return binDirectory() + "/catkin/colcon_make_aiil"

'''
Root directory of Codebase
'''
def checkoutDirectory():
    return getEnvParameter('AIIL_CHECKOUT_DIR')

'''
Location of configuration files for ROSBot
'''
def configDirectory():
    return checkoutDirectory() + "/config"

'''
Location of docker directory
'''
def dockerDirectory():
    return checkoutDirectory() + "/docker"

'''
Retrieve Ennvironment parameter from executing shell
'''
def getEnvParameter(param, check=False):
    value = ""
    if param in os.environ:
        value = os.environ[param]
    else :
        # If checking mode, return the empty string, otherwise error.
        if check:
            print_warning("Cannot load env param (" + param + ")")
        else :
            print_error("Cannot load env param (" + param + 
                        "). RedbackBots Bash Config probably not loaded correctly."+
                        " Source <checkout_dir>/.bashrc")
            exit()
    return value

'''
Get the keys from the config block excluding the default block
'''
def getKeys(config):
    keys = [key for key in config.keys() if key != configparser.DEFAULTSECT]
    return keys

'''
Load file by name from root of config directory
'''
def loadConfig(file):
    fullPath = configDirectory() + "/" + file
    if not os.path.exists(fullPath):
        print_error("Cannot locate config file: " + fullPath)
        return None
    
    config = loadConfigFile(fullPath)
    return config

'''
Load file by full path
'''
def loadConfigFile(file):
    config = configparser.ConfigParser()
    config.read(file)
    return config

'''
Rosbot root home location
'''
def rosbotHome():
    return "/home/husarion"

'''
Location of the ROSBot Melodic workspace
'''
def rosbotMelodicWorkspace():
    return checkoutDirectory() + "/melodic_workspace"

def rosbotHumbleWorkspace():
    return checkoutDirectory() + "/humble_workspace"

'''
Location of ssh_config file
'''
def sshLocalConfig():
    return configDirectory() + "/ssh_config"

