import launch
import launch.actions
import launch.substitutions
from launch_ros.actions import Node
ns = 'pi_car'
def generate_launch_description():
    return launch.LaunchDescription([
        Node(
            package='pi_car',
            namespace=ns,
            executable='pi_car'
        ),

        Node(
            package='usb_camera_driver',

            # package='image_tools',
            # executable='cam2image',
            namespace=ns, 
            executable='usb_camera_driver_node',
            parameters=[{
                'camera_calibration_file': 'file:///home/ubuntu/.ros/camera_info/camera.yaml',
            }
        ]

        )



    ])