- bringup
    - config (holds yaml files)
    - launch (holds launch.py files for general robot and controller bringup. The ignition key and assembly line)
- description
    - launch (holds robot specific launch files)
    - ros2_control (holds ros2_control specific macros for xacro)
    - urdf (holds robot urdf.xacro main files)
- doc (holds documentation and imagery)
- hardware (holds hardware-specific code?)
- test (holds all test files to make sure everything is working as intended)

Why Separate bringup and description launch files?
Dependency Management: A different team working on motion planning might only need to know what your robot looks like. They can install just your lightweight description package without needing to install Gazebo and all your ros2_control dependencies.

Clarity: It separates concerns. If you want to change the robot's physical shape, you go to the description package. If you want to change how it's controlled or which simulator it runs in, you go to the bringup package.

Reusability: You can write a different bringup package for a different simulator, or for the real hardware, all while using the exact same description package.