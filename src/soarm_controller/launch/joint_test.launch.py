from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration
import os
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    #Retrieve file path of URDF file
    model_arg = DeclareLaunchArgument(
        name="model", 
        default_value=os.path.join(get_package_share_directory("soarm_description"), "urdf", "soarm.urdf.xacro"),
        description="Absolute path to robot URDF file"
        )
    
    #Need to convert xacro to readable URDF, using path defined earlier DeclareLaunchArgument(name="model"), Command specifies conversion
    robot_description = ParameterValue(Command(["xacro ", LaunchConfiguration("model")])) #SPACE AFTER XACRO IS IMPORTANT


    #Typical Syntax for starting Nodes, these are most important
    robot_state_pub = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{
            'robot_description': robot_description, # Use the variable name here!
            'use_sim_time': True
        }]
    )

    joint_tester = Node(
        package="soarm_scripts",
        executable="joint_tester",
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time' : True
        }]
    )


    #LaunchDescription is a list of instructions to execute when launching file. Define objects, and stick into list of args
    return LaunchDescription([
        model_arg,
        robot_state_pub,
        joint_tester
    ])