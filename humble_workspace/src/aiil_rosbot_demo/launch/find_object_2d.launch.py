from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable, LogInfo
from launch.substitutions import LaunchConfiguration, EnvironmentVariable
from launch_ros.actions import Node

def generate_launch_description():
   
    return LaunchDescription([

        SetEnvironmentVariable('RCUTILS_LOGGING_USE_STDOUT', '1'),
        SetEnvironmentVariable('RCUTILS_LOGGING_BUFFERED_STREAM', '0'),

        # Launch arguments
        # Set to false when running after training is finished
        DeclareLaunchArgument('gui', default_value='true', description='Launch GUI.'),
        
        # Our default camera topic. If streaming images, consider using the compressed image instead
        DeclareLaunchArgument('image_topic', default_value='/camera/color/image_raw', description='Image topic to subscribe to.'),
        #DeclareLaunchArgument('image_topic', default_value='/camera/color/image_raw/compressed', description='Image topic to subscribe to.'),
        
        # Path where you have saved the existing trained images
        # Uses the path to the AIIL Workspace, but this could be set to anywhere
        LogInfo(msg=('AIIL_CHECKOUT_DIR, ', EnvironmentVariable(name='AIIL_CHECKOUT_DIR'))),
        DeclareLaunchArgument('objects_path', 
                               default_value=[EnvironmentVariable(name='AIIL_CHECKOUT_DIR'),'/humble_workspace/objects'],
                               description='Directory containing objects to load on initialization.'),
        
        # Find Object 2D Setting. By default just use the standard settings, but you can copy and tweak this file if you wish
        DeclareLaunchArgument('settings_path', default_value='~/.ros/find_object_2d.ini', description='Config file.'),      

        # Nodes to launch
        Node(
            package='find_object_2d',
            executable='find_object_2d',
            output='screen',
            parameters=[{
              'gui': LaunchConfiguration('gui'),
              'objects_path': LaunchConfiguration('objects_path'),
              'settings_path': LaunchConfiguration('settings_path')
            }],
            remappings=[
                ('image', LaunchConfiguration('image_topic'))
            ]), 
    ])
