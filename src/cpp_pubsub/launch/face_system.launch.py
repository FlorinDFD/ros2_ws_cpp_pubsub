from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([

        # Nodul 1 - Camera
        Node(
            package='cpp_pubsub',
            executable='image_publisher',
            name='camera_node',
            output='screen'
        ),

        # Nodul 2 - Kinect-like detector
        Node(
            package='cpp_pubsub',
            executable='image_subscriber',
            name='kinect_node',
            output='screen'
        ),

        # Nodul 3 - Face Recognizer
        Node(
            package='cpp_pubsub',
            executable='face_recognizer',
            name='face_recognizer_node',
            output='screen'
        ),
    ])
