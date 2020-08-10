import launch
import launch.actions
import launch.substitutions
from launch_ros.actions import Node

def generate_launch_description():
    return launch.LaunchDescription([
        Node(
            package='pi_car',
            namespace='pi_car',
            executable='pi_car',
            name='pi_car'
        ),

        Node(
            package='image_tools',
            namespace='pi_car',
            executable='cam2image',
            name='cam2image'
        )



    ])