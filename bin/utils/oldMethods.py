def setupBuildHusarion(configRobots, configComputers, configSoftware):
    # Create and configure CMake
    print_status("Configure & Build Husarion Workspace")

    # Setup Husarion Repos
    setupHusarionRepos(configRobots, configComputers, configSoftware)
    
    # Fix melodic issue
    if False and setupRobot:
        setupHusarionFixes(configRobots, configComputers)

    # Execute standalone catkin_make script for husarion_ws
    print_status("Building Husarion Workspace")
    binDir = cfg.binDirectory()
    script = binDir + "/catkin/catkin_make_husarion"
    shell.exec(script, hideOutput=False)

def setupHusarionRepos(configRobots, configComputers, configSoftware):
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
                os.chdir(AIIL_CHECKOUT_DIR)

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
                os.chdir(AIIL_CHECKOUT_DIR)

    # Install NodeJS and NPM packages for route_admin_panel repository
    if configRepos.has_section("route_admin_panel"):
        print_status("Installing NodeJS & NPM packages for Husarion route_admin_panel repo")
        panelDir = HUSARION_CHECKOUT_DIR + "/src/route_admin_panel/nodejs"
        
        # Configure NPM packages to install
        packages = ""
        npmSoftware = configSoftware['NPM']
        for pkg in npmSoftware:
            if npmSoftware.getboolean(pkg):
                packages += " " + pkg
        
        # Do install
        print_subitem("Installing in " + panelDir)
        if os.path.exists(panelDir):
            os.chdir(panelDir)
            # NPM packages
            command = "npm install " + packages
            print_subitem(command)
            shell.exec(command, hideOutput=False)
            # NPM general install
            command = "npm install "
            print_subitem(command)
            shell.exec(command, hideOutput=False)
            os.chdir(AIIL_CHECKOUT_DIR)

def setupHusarionFixes(configRobots, configComputers):
    rosversion = configRobots[setupRobotName]['rosversion']
    if rosversion == 'melodic':
        print_status("Fixing issues with astra_camera OpenNi Log configuration")
        filename = HUSARION_CHECKOUT_DIR + "/src/astra_camera/ros_astra_camera-master/include/openni2_redist/x64/OpenNI.ini"
        if os.path.exists(filename):
            command = "sed -i 's/LogToConsole=[0-9]/LogToConsole=0/' " + filename
            print_subitem("Executing: " + command)
            shell.exec(command, hideOutput=True)