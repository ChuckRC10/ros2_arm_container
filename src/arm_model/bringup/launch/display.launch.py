import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():

    package_name='arm_model'
    # Get paths to config files
    xacro_file = os.path.join(get_package_share_directory(package_name), "description", "urdf", "arm_model.urdf.xacro")
    rviz_config_file = os.path.join(get_package_share_directory(package_name), "rviz", "display.rviz")
    world_sdf_file = os.path.join(get_package_share_directory(package_name), "gazebo", "worlds", "arm_model.sdf")

    # Get robot description from xacro
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            xacro_file,
        ]
    )
    robot_description = {"robot_description": robot_description_content}

    # Launch Gazebo directly as a process
    gazebo = ExecuteProcess(
        cmd=['gz', 'sim', '-r', world_sdf_file],
        additional_env={'GZ_SIM_SYSTEM_PLUGIN_PATH': '/opt/ros/jazzy/lib'},
        output='screen'
    )

    # Node for robot_state_publisher
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[robot_description],
    )

    # Node for RViz2
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_file],
    )

    # Node to spawn the robot in Gazebo
    spawn_entity_node = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=['-topic', 'robot_description', '-name', 'arm_model'],
        output='screen',
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

    # Node to load key to joint trajectory bridge (streaming_bridge_node.py)
    key_to_joint_traj_bridge = Node(
        package='arm_model',
        executable='streaming_bridge_node',
        output='screen',
    )

    # Ensure RViz starts after the joint_state_broadcaster is ready
    delay_rviz_after_joint_state_broadcaster_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[rviz_node],
        )
    )

    bridge_params = os.path.join(get_package_share_directory(package_name), 'bringup', 'config', 'gz_bridge.yaml')
    ros_gz_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=[
            '--ros-args',
            '-p',
            f'config_file:={bridge_params}',
        ]
    )

    return LaunchDescription([
        gazebo,
        robot_state_publisher_node,
        spawn_entity_node,
        joint_state_broadcaster_spawner,
        joint_trajectory_controller_spawner,
        delay_rviz_after_joint_state_broadcaster_spawner,
        ros_gz_bridge,
        key_to_joint_traj_bridge,
    ])
