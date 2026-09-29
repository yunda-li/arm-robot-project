import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch.conditions import UnlessCondition



def generate_launch_description():
    soarm_description_dir = get_package_share_directory("soarm_description")

    is_sim_arg = DeclareLaunchArgument(
        name="is_sim",
        #CHANGE HERE FOR SIM VS REAL
        # default_value="true",
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

    pick_place_server = Node(
        package="soarm_scripts",
        executable="pick_place_server_node",
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': is_sim
        }]
    )

    pick_place_client = Node(
        package="soarm_scripts",
        executable="pick_place_client_node",
        parameters=[{
            'use_sim_time': is_sim
        }]
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

    return LaunchDescription([
        is_sim_arg,
        model_arg,
        robot_state_pub,
        pick_place_server,
        pick_place_client,
        controller_manager,
        joint_state_broadcaster_spawner,
        arm_controller_spawner,
        gripper_controller_spawner
    ])