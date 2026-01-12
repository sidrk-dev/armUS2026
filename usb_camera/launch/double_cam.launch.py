from launch import LaunchDescription
from launch_ros.actions import Node
import os


def generate_launch_description():
    ld = LaunchDescription()

    cam_1_node = Node(
        package='image_tools',
        executable='cam2image',
        name='front_camera',
        output='log',
        remappings=[
            ('image', 'front/image')
        ],
        parameters=[
            #half resolution
            {'device_id': 0},
            {'height': 240},
            {'width': 320},
            {'frequency': 1.0},
            {'reliability': 'best_effort'}
        ]
    )

    cam_2_node = Node(
        package='image_tools',
        executable='cam2image',
        name='rear_camera',
        output='screen',
        remappings=[
            ('image', 'rear/image')
        ],
        parameters=[
            #full resolution
            {'device_id': 2},
            {'height': 480},
            {'width': 640},
            {'frequency': 1.0},
            {'reliability': 'best_effort'}
        ]
    )

    """front_trans_node = Node(
        package='image_transport',
        executable='republish',
        arguments=["raw"],
        remappings=[('in', 'front/image'),
                    ('out', 'front/image/compress')
                    ],
        parameters=[
            {'reliability': 'best_effort'}
                    ]
    )

    rear_trans_node = Node(
        package='image_transport',
        executable='republish',
        arguments=["raw"],
        remappings=[('in', 'rear/image'),
                    ('out', 'rear/image/compress')
                    ],
        parameters=[
            {'reliability': 'best_effort'}
                    ]
    )"""    

    ld.add_action(cam_1_node)
    #ld.add_action(front_trans_node)
    ld.add_action(cam_2_node)
    #ld.add_action(rear_trans_node)
# 複数ノードを追加する場合は，configN,nodeNを作ってld.add_action(nodeN)?

    return ld
