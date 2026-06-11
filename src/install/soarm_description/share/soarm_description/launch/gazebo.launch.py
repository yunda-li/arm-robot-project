from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
import os
from ament_index_python.packages import get_package_share_directory
from pathlib import Path


def generate_launch_description():

    soarm_description_dir = get_package_share_directory("soarm_description")
    #Retrieve file path of URDF file
    model_arg = DeclareLaunchArgument(
        name="model", 
        default_value=os.path.join(soarm_description_dir, "urdf", "soarm.urdf.xacro"),
        description="Absolute path to robot URDF file"
        )
    
    #Communicate to Gazebo where model and meshes should be
    gazebo_resource_path = SetEnvironmentVariable(
        name="GZ_SIM_RESOURCE_PATH",
        value=[
            str(Path(soarm_description_dir).parent.resolve())
        ]
    )


    # ros_distro = os.environ["ROS_DISTRO"]
    # physics_engine = "" if ros_distro == "humble" else "--physics-engine gz-physics-bullet-featherstone-plugin"
    
    #Need to convert xacro to readable URDF, using path defined earlier DeclareLaunchArgument(name="model"), Command specifies conversion
    robot_description = ParameterValue(
        Command(["xacro ", LaunchConfiguration("model")]),
        value_type=str
    )#SPACE AFTER XACRO IS IMPORTANT


    #Typical Syntax for starting Nodes, these are most important
    robot_state_pub = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{
            'robot_description': robot_description,
            "use_sim_time" : True
        }]
    )

    #Start empty Gazebo simulation, use another launch file within this launch file
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [os.path.join(get_package_share_directory("ros_gz_sim"), "launch"), "/gz_sim.launch.py"]

        ),
        launch_arguments=[("gz_args", [" -v 4 -r empty.sdf "])] #NEED SPACE AFTER DUE TO TERMINAL COMMAND

    )

    #Spawn Robot Entity
    gz_spawn_entity = Node(
        package="ros_gz_sim",
        executable="create",
        output="screen",
        arguments=["-topic", "robot_description", "-name", "soarm"]
    )

    #Allow ROS2 to read messages from Gazebo, without camera
    # gz_ros2_bridge = Node(
    #     package="ros_gz_bridge",
    #     executable="parameter_bridge",
    #     arguments=[
    #         "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock",
    #     ]
    # )

    #With camera
    # gz_ros2_bridge = Node(
    #     package="ros_gz_bridge",
    #     executable="parameter_bridge",
    #     arguments=[
    #         "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock",
    #         "/image_raw@sensor_msgs/msg/Image[gz.msgs.Image",
    #         "/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo",
    #     ]
    # )

    return LaunchDescription([
        model_arg,
        gazebo_resource_path,
        robot_state_pub,
        gazebo,
        gz_spawn_entity,
        # gz_ros2_bridge
    ])