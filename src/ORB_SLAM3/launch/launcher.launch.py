from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription()

    orbslam_node_mono= Node(
        package='orb_slam3_ros',
        executable='orb_slam3_ros_node',
    )

    ld.add_action(orbslam_node_mono)
    


    return ld