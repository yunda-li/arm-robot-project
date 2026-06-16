import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    soarm_description_dir = get_package_share_directory("soarm_description")

    is_sim_arg = DeclareLaunchArgument(
        name="is_sim",
        #CHANGE HERE
        default_value="false",
        description="Toggle between simulation (True) and real hardware (False)"
    )

    is_sim = LaunchConfiguration("is_sim")

    model_arg = DeclareLaunchArgument(
        name="model", 
        default_value=os.path.join(soarm_description_dir, "urdf", "soarm.urdf.xacro"),
        description="Absolute path to robot URDF file"
    )
    
    robot_description = ParameterValue(
        Command([
            "xacro ", 
            LaunchConfiguration("model"),
            " is_sim:=", is_sim
        ]),
        value_type=str
    )

    robot_state_pub = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': is_sim
        }]
    )

    joint_tester = Node(
        package="soarm_scripts",
        executable="joint_tester",
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': is_sim
        }]
    )

    return LaunchDescription([
        is_sim_arg,
        model_arg,
        robot_state_pub,
        joint_tester
    ])