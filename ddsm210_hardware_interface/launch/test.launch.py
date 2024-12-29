from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # Declare arguments
    declared_arguments = []
    global enable_debug
    
    # Initialize Arguments
    robot_description_package = "ddsm210_hardware_interface"
    
    # Get paths to config files
    robot_description_config = Command(
        [
            FindExecutable(name='xacro'), ' ',
            PathJoinSubstitution(
                [FindPackageShare(robot_description_package), 'urdf', 'test.urdf.xacro']
            )
        ]
    )
    
    controllers_file = PathJoinSubstitution(
        [FindPackageShare(robot_description_package), 'config', 'test_controllers.yaml']
    )

    # Define nodes
    robot_state_pub_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description_config}]
    )
    enable_debug = False
    prefix = 'gdbserver localhost:2000' if enable_debug else None
    print(prefix)
    controller_manager_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        prefix=prefix,
        parameters=[
            {'robot_description': robot_description_config},
            controllers_file
        ],
        output='screen',
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster"],
    )

    velocity_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["velocity_controller"],
    )
    
    nodes = [
        robot_state_pub_node,
        controller_manager_node,
        joint_state_broadcaster_spawner,
        velocity_controller_spawner
    ]

    return LaunchDescription(declared_arguments + nodes)