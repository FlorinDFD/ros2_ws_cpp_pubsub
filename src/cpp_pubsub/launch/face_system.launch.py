from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(package='cpp_pubsub', executable='image_publisher', name='camera_node', output='screen'),
        Node(package='cpp_pubsub', executable='image_subscriber', name='kinect_node', output='screen'),
        Node(package='cpp_pubsub', executable='face_recognizer', name='face_recognizer', output='screen'),
    ])
