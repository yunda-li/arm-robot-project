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
            'use_sim_time': False
        }]
    )

    
    joint_state_publisher_gui = Node(
        package = "joint_state_publisher_gui",
        executable="joint_state_publisher_gui"
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        #log message
        output="screen",
        #Allows loading of saved robot configuration from earlier, with tf, RobotModel loaded in rviz
        arguments=["-d", os.path.join(get_package_share_directory("soarm_description"), "rviz", "display.rviz")]
    )

    #LaunchDescription is a list of instructions to execute when launching file. Define objects, and stick into list of args
    return LaunchDescription([
        model_arg,
        robot_state_pub,
        joint_state_publisher_gui,
        rviz_node
    ])