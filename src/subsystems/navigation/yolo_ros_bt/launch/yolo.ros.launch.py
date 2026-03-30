from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

ARGUMENTS = [
    DeclareLaunchArgument('use_sim_time', default_value='false'),
    DeclareLaunchArgument('conf_thres', default_value='0.5'),
]

def generate_launch_description():
    yolo_node = Node(
        package='yolo_ros_bt',      # change
        executable='yolo_node',     # change
        name='yolo_node',
        parameters=[
            {'use_sim_time': LaunchConfiguration('use_sim_time')},
            {'conf_thres': LaunchConfiguration('conf_thres')},
        ],
        output='screen'
    )

    ld = LaunchDescription(ARGUMENTS)
    ld.add_action(yolo_node)
    return ld