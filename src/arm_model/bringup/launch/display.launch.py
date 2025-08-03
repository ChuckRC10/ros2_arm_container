import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
import xacro

def generate_launch_description():

    # Get the package share directory
    pkg_share = get_package_share_directory('arm_model')

    # Define the path to the xacro file
    xacro_file = os.path.join(pkg_share, 'description', 'urdf', 'arm_model.urdf.xacro')
    controllers_file = os.path.join(pkg_share, 'bringup', 'config', 'arm_controller.yaml')

    # Process the xacro file to generate the robot description
    robot_description_config = xacro.process_file(xacro_file)
    robot_desc = robot_description_config.toxml()

    # Define the path to the rviz config file
    rviz_config_file = os.path.join(pkg_share, 'rviz', 'display.rviz')

    # Define the path to the gazebo world file
    world_sdf_file = os.path.join(pkg_share, 'gazebo', 'arm_model.sdf')

    # Node for robot_state_publisher
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_desc}]
    )

    # Node for RViz2
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_file]
    )

    # Launch Gazebo
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')),
        launch_arguments={'gz_args': world_sdf_file}.items(),  
    )

    # Node to spawn the robot in Gazebo
    spawn_entity_node = Node(
        package='ros_gz_sim', 
        executable='create',
        arguments=['-topic', 'robot_description', '-name', 'arm_model'],
        output='screen'
    )
    
    # Node for the ros2_control controller manager
    controller_manager_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[{'robot_description': robot_desc}, controllers_file],
        output="screen",
    )

    # Node to load the Joint State Broadcaster
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
    )

    # Node to load the Joint Trajectory Controller
    joint_trajectory_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_trajectory_controller", "--controller-manager", "/controller_manager"],
    )


    return LaunchDescription([
        gazebo,
        robot_state_publisher_node,
        spawn_entity_node,
        controller_manager_node,
        joint_state_broadcaster_spawner,
        joint_trajectory_controller_spawner,
        rviz_node
    ])
