from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package="image_processing_pkg",
            executable="image_crop_server_node",
            name="server"
        ),
        Node(
            package='image_processing_pkg',
            executable='image_crop_client_node',
            name='client',
            parameters=[{
                "image_name": "image_4.jpeg",
                "iterations_num": 3,
                "cropped_image_path":"/home/YOUR_USER/ros2_ws/src/image_processing_pkg/res/cropped"
            }]
        )
    ])