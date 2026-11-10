import os
from launch import LaunchDescription
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
from launch_ros.parameter_descriptions import ParameterValue
from launch.conditions import UnlessCondition, IfCondition



def generate_launch_description():
    soarm_description_dir = get_package_share_directory("soarm_description")
    # moveit_config = MoveItConfigsBuilder("soarm", package_name="soarm_moveit").to_moveit_configs()

    is_sim_arg = DeclareLaunchArgument(
        name="is_sim",
        #CHANGE HERE
        default_value="false",
        description="Toggle between simulation (True) and real hardware (False)"
    )

    is_sim = LaunchConfiguration("is_sim")

    moveit_config = (
        MoveItConfigsBuilder("soarm", package_name="soarm_moveit")
        .robot_description(mappings={"is_sim": is_sim})
        .to_moveit_configs()
    )

    robot_description = ParameterValue(
        Command(
            [
                "xacro ",
                os.path.join(
                    soarm_description_dir,
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

    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[moveit_config.to_dict(), {"use_sim_time": is_sim}],
    )
    test_node = Node(
        package="soarm_scripts_ik",
        executable="servo_test",
        output="screen",
        parameters=[moveit_config.to_dict(), {'use_sim_time': is_sim}]
    )

    return LaunchDescription([
        is_sim_arg, 
        robot_state_publisher_node,
        controller_manager,
        joint_state_broadcaster_spawner,
        arm_controller_spawner,
        move_group_node,
        test_node])