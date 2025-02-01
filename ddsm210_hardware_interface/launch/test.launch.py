# Copyright 2025 Alessio Morale <alessiomorale-at-gmail.com>
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in
# all copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
# THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
# THE SOFTWARE.


from launch import LaunchDescription
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution
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
            FindExecutable(name="xacro"),
            " ",
            PathJoinSubstitution(
                [FindPackageShare(robot_description_package), "urdf", "test.urdf.xacro"]
            ),
        ]
    )

    controllers_file = PathJoinSubstitution(
        [FindPackageShare(robot_description_package), "config", "test_controllers.yaml"]
    )

    # Define nodes
    robot_state_pub_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[{"robot_description": robot_description_config}],
    )
    enable_debug = False
    prefix = "gdbserver localhost:2000" if enable_debug else None
    print(prefix)
    controller_manager_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        prefix=prefix,
        parameters=[{"robot_description": robot_description_config}, controllers_file],
        output="screen",
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
        velocity_controller_spawner,
    ]

    return LaunchDescription(declared_arguments + nodes)
