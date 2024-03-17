from setuptools import find_packages, setup
from glob import glob
import os

package_name = 'aiil_gazebo'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',['resource/' + package_name]),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch')),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        #(os.path.join('share', package_name, 'models'), glob('models/*')),
        #(os.path.join('share', package_name, 'worlds'), glob('worlds/*')),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='AI Innovation Lab, RMIT University',
    maintainer_email='timothy.wiley@rmit.edu.au',
    description='The aiil_gazebo package',
    license='RMIT IP - Not for distribution',
    # tests_require=['pytest'],
    # entry_points={
    #     'console_scripts': [
    #         f"cmd_vel = {package_name}.cmd_vel:main",
    #     ],
    # },
)
