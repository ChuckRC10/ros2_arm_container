import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():

    # Get paths to config files
    xacro_file = PathJoinSubstitution(
        [FindPackageShare("arm_model"), "description", "urdf", "arm_model.urdf.xacro"]
    )
    controllers_file = PathJoinSubstitution(
        [FindPackageShare("arm_model"), "bringup", "config", "arm_controller.yaml"]
    )
    rviz_config_file = PathJoinSubstitution(
        [FindPackageShare("arm_model"), "rviz", "display.rviz"]
    )
    world_sdf_file = PathJoinSubstitution(
        [FindPackageShare("arm_model"), "gazebo", "arm_model.sdf"]
    )

    # Get robot description from xacro
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            xacro_file,
        ]
    )
    robot_description = {"robot_description": robot_description_content}


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

    # Launch Gazebo
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [os.path.join(get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')]
        ),
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
        parameters=[robot_description, controllers_file],
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

    # Ensure RViz starts after the joint_state_broadcaster is ready
    delay_rviz_after_joint_state_broadcaster_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[rviz_node],
        )
    )

    return LaunchDescription([
        gazebo,
        robot_state_publisher_node,
        spawn_entity_node,
        controller_manager_node,
        joint_state_broadcaster_spawner,
        joint_trajectory_controller_spawner,
        delay_rviz_after_joint_state_broadcaster_spawner,
    ])
