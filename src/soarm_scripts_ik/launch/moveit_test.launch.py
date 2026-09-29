from launch import LaunchDescription
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration

def generate_launch_description():
    moveit_config = MoveItConfigsBuilder("soarm", package_name="soarm_moveit").to_moveit_configs()

    is_sim_arg = DeclareLaunchArgument(
        name="is_sim",
        #CHANGE HERE
        default_value="true",
        description="Toggle between simulation (True) and real hardware (False)"
    )

    is_sim = LaunchConfiguration("is_sim")


    test_node = Node(
        package="soarm_scripts_ik",
        executable="moveit_test",
        output="screen",
        parameters=[moveit_config.to_dict(), {'use_sim_time': is_sim}]
    )

    return LaunchDescription([is_sim_arg, test_node])