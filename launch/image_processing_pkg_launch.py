from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package="image_processing_pkg",
            executable="image_crop_server_node",
            name="server",
            parameters=[{
                "image_path": "images",
                "tries": "1",
                "cropped_image_path":"cropped"
            }]
        ),
        """Node(
            package='image_processing_pkg',
            executable='image_crop_client_node',
            name='client',
            parameters=[{
                "image_path": "earth",
                "tries": "1",
                "cropped_image_path":""
            }]
        )"""
    ])