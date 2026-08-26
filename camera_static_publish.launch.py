from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments = ['-0.022542648636446606', '-0.04542167475614861', '0.03449884974187979', 
                         '0.023201672357022737', '-0.027023357208746875', '0.7093108444496831', '0.7039954165412466', 
                         'panda_hand_tcp', 'camera_front_link']
        )
    ])
