from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    model = LaunchConfiguration("model")
    use_joint_state_publisher = LaunchConfiguration("use_joint_state_publisher")
    use_rviz = LaunchConfiguration("use_rviz")

    return LaunchDescription([
        DeclareLaunchArgument(
            "model",
            default_value=PathJoinSubstitution([
                FindPackageShare("amr_description"),
                "urdf",
                "amr.urdf",
            ]),
            description="Path to the AMR URDF file.",
        ),
        DeclareLaunchArgument(
            "use_joint_state_publisher",
            default_value="false",
            description="Start joint_state_publisher if it is installed.",
        ),
        DeclareLaunchArgument(
            "use_rviz",
            default_value="true",
            description="Start RViz2 for visual inspection.",
        ),
        Node(
            package="joint_state_publisher",
            executable="joint_state_publisher",
            name="joint_state_publisher",
            output="screen",
            condition=IfCondition(use_joint_state_publisher),
        ),
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            name="robot_state_publisher",
            output="screen",
            parameters=[{
                "robot_description": ParameterValue(Command(["cat ", model]), value_type=str),
            }],
        ),
        Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            output="screen",
            arguments=["-d", PathJoinSubstitution([
                FindPackageShare("amr_description"),
                "config",
                "display.rviz",
            ])],
            condition=IfCondition(use_rviz),
        ),
    ])
