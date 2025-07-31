import os
from glob import glob
from setuptools import setup

package_name = 'arm_model'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # This line installs all .launch.py files
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        # This line installs all .xacro files from the urdf directory
        (os.path.join('share', package_name, 'urdf'), glob('urdf/*.xacro')),
        # This line installs all .rviz files
        (os.path.join('share', package_name, 'rviz'), glob('rviz/*.rviz')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='your_name',
    maintainer_email='your_email@example.com',
    description='A package for the arm model.',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        ],
    },
)
