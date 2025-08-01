import os
from glob import glob
from setuptools import setup

package_name = 'arm_model'

setup(
    name=package_name,
    version='0.0.0',
    # This should be empty if you have no python modules in a subfolder
    packages=[], 
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        
        # Corrected paths for your structure
        (os.path.join('share', package_name, 'launch'), glob('bringup/launch/*.launch.py')),
        (os.path.join('share', package_name, 'urdf'), glob('description/urdf/*.xacro')),
        (os.path.join('share', package_name, 'description/ros2_control'), glob('description/ros2_control/*')),
        (os.path.join('share', package_name, 'rviz'), glob('rviz/*.rviz')),
        (os.path.join('share', package_name, 'config'), glob('bringup/config/*.yaml'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Charles Curione',
    maintainer_email='ccurione@purdue.edu',
    description='A package for the arm model.',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [],
    },
)
