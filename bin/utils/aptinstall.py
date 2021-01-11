#! /usr/bin/env python3

'''
Use python bindings around apt-install to configure software
'''

import apt
import sys

import utils.echo as echo

cache = apt.cache.Cache()
#cache.update()
cache.open()

def aptinstall(pkg_name):

    pkg = cache[pkg_name]
    installed = False
    if pkg.is_installed:
        echo.print_subitem("{pkg_name} already installed".format(pkg_name=pkg_name))
        installed = True
    else:
        pkg.mark_install()

        try:
            cache.commit()
            installed = True
        except arg:
            echo.print_error("Installation failed [{err}]".format(err=str(arg)))
    
    return installed

