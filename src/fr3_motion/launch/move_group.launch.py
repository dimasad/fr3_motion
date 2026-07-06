"""Bring up MoveIt's move_group node for the FR3, providing the /compute_ik service.

franka_fr3_moveit_config's own move_group.launch.py never declares its launch
arguments (robot_ip, namespace, arm_prefix, ...), so `ros2 launch
franka_fr3_moveit_config move_group.launch.py ...` fails on the command line.
It only works when included from another launch file that sets those
LaunchConfigurations first -- which is what this file does.

This only brings up move_group. Bring up the robot separately with
`franka_bringup franka.launch.py`, then load/activate controllers as usual.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    robot_ip_arg = DeclareLaunchArgument(
        "robot_ip",
        default_value="172.16.0.2",
        description="Hostname or IP address of the robot",
    )
    use_fake_hardware_arg = DeclareLaunchArgument(
        "use_fake_hardware",
        default_value="false",
        description="Use fake hardware (for testing without a robot)",
    )
    load_gripper_arg = DeclareLaunchArgument(
        "load_gripper",
        default_value="true",
        description="Whether the Franka Hand is attached",
    )
    fake_sensor_commands_arg = DeclareLaunchArgument(
        "fake_sensor_commands",
        default_value="false",
        description="Fake sensor commands (only relevant with use_fake_hardware)",
    )

    move_group = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("franka_fr3_moveit_config"), "launch", "move_group.launch.py"]
            )
        ),
        launch_arguments={
            "robot_ip": LaunchConfiguration("robot_ip"),
            "use_fake_hardware": LaunchConfiguration("use_fake_hardware"),
            "load_gripper": LaunchConfiguration("load_gripper"),
            "fake_sensor_commands": LaunchConfiguration("fake_sensor_commands"),
            "namespace": "",
            "arm_prefix": "",
        }.items(),
    )

    return LaunchDescription(
        [
            robot_ip_arg,
            use_fake_hardware_arg,
            load_gripper_arg,
            fake_sensor_commands_arg,
            move_group,
        ]
    )
