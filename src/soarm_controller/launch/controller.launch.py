import os
from launch import LaunchDescription
# These two for launching other launch files, along with IfCondition below
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch.substitutions import Command, LaunchConfiguration
from launch.conditions import UnlessCondition, IfCondition
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    soarm_description_dir = get_package_share_directory("soarm_description")
    # soarm_controller_dir = get_package_share_directory("soarm_controller")

    #Whether gazebo or real hardware
    is_sim = LaunchConfiguration("is_sim")
    
    is_sim_arg = DeclareLaunchArgument(
        "is_sim",
        default_value="True"
    )

    gazebo_launch = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(soarm_description_dir, "launch", "gazebo.launch.py")
            ),
            condition=IfCondition(is_sim)
        ) 



    robot_description = ParameterValue(
        Command(
            [
                "xacro ",
                os.path.join(
                    get_package_share_directory("soarm_description"),
                    "urdf",
                    "soarm.urdf.xacro",
                ),
                " is_sim:=", is_sim
            ]
        ),
        value_type=str,
    )

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{"robot_description": robot_description,
                    "use_sim_time": is_sim}],
                    #  "use_sim_time": False}],
        # condition=UnlessCondition(is_sim),
    )

    #Manage and supervise controllers
    controller_manager = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[
            {"robot_description": robot_description,
             "use_sim_time": is_sim},
            os.path.join(
                get_package_share_directory("soarm_controller"),
                "config",
                "soarm_controllers.yaml",
            ),
        ],
        condition=UnlessCondition(is_sim),
    )

    #Each separate controller needs to be loaded in as separate node
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager",
            "/controller_manager",
        ],
    )

    arm_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["arm_controller", "--controller-manager", "/controller_manager"],
    )

    gripper_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["gripper_controller", "--controller-manager", "/controller_manager"],
    )

    return LaunchDescription(
        [
            is_sim_arg,
            gazebo_launch,
            robot_state_publisher_node,
            controller_manager,
            joint_state_broadcaster_spawner,
            arm_controller_spawner,
            gripper_controller_spawner,
        ]
    )