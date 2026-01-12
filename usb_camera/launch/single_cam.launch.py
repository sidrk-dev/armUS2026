from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
#from rclpy.qos import QoSProfile, QoSReliabilityPolicy
import os

pkg_name1 = 'front_camera'

def generate_launch_description():
    ld = LaunchDescription()

    ld.add_action(DeclareLaunchArgument('front_id', default_value='0', description='Front camera device ID'))
    ld.add_action(DeclareLaunchArgument('rear_id', default_value='2', description='Rear camera device ID'))

    front_cam_node = Node(
        package='image_tools',
        executable='cam2image',
        name='front_camera',
        output='log',
        remappings=[
            ('image', 'front/image')
        ],
        parameters=[
            {'reliability': 'reliable'}, # Can be changed to best_effort, confirmed
            {'history': 'keep_last'},
            {'depth': 10},
            {'device_id': LaunchConfiguration('front_id')},
            {'height': 240},
            {'width': 320},
            {'frequency': 2.0},
        ]
    )


    rear_cam_node = Node(
        package='image_tools',
        executable='cam2image',
        name='rear_camera',
        output='screen',
        remappings=[
            ('image', 'rear/image')
        ],
        parameters=[
            {'reliability': 'reliable'}, # Can be changed to best_effort, confirmed
            {'history': 'keep_last'},
            {'depth': 10},
            {'device_id': LaunchConfiguration('rear_id')},
            {'height': 240},
            {'width': 320},
            {'frequency': 2.0},
        ]
    )


    front_trans_node = Node(
        package='image_transport',
        executable='republish',
        name='front_trans',
        arguments=["raw", "raw"],
        remappings=[('in', 'front/image'),
                    ('out', 'front/image/compress')
                    ]
    )

    front_qos_node = Node(
        package='usb_camera',
        executable='qos_set',
        name='front_qos',
        remappings=[('image', 'front/image/compress'),
                    ('image/qos_set', 'front/image/compress/qos_set')
                    ]
    )


    rear_trans_node = Node(
        package='image_transport',
        executable='republish',
        name='rear_trans',
        arguments=["raw", "raw"],
        remappings=[('in', 'rear/image'),
                    ('out', 'rear/image/compress')
                    ]
    )    

    rear_qos_node = Node(
        package='usb_camera',
        executable='qos_set',
        name='rear_qos',
        remappings=[('image', 'rear/image/compress'),
                    ('image/qos_set', 'rear/image/compress/qos_set')
                    ]
    )

    ld.add_action(front_cam_node)
    ld.add_action(front_trans_node)
    ld.add_action(front_qos_node)
    ld.add_action(rear_cam_node)
    ld.add_action(rear_trans_node)
    ld.add_action(rear_qos_node)
# To add multiple nodes, create configN, nodeN and use ld.add_action(nodeN)?

    return ld
