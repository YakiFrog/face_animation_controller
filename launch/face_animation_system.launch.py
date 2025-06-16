#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    """
    Face Animation Controller システム全体を起動するlaunchファイル
    """
    
    # パラメータの宣言
    face_api_url_arg = DeclareLaunchArgument(
        'face_api_url',
        default_value='http://localhost:8080',
        description='Face animation HTTP API URL'
    )
    
    topic_name_arg = DeclareLaunchArgument(
        'topic_name',
        default_value='/face_expression',
        description='ROS2 topic name for face expressions'
    )
    
    port_arg = DeclareLaunchArgument(
        'port',
        default_value='8080',
        description='HTTP server port'
    )
    
    host_arg = DeclareLaunchArgument(
        'host',
        default_value='0.0.0.0',
        description='HTTP server host'
    )
    
    # HTTP Server ノード
    http_server_node = Node(
        package='face_animation_controller',
        executable='face_http_server',
        name='face_http_server',
        parameters=[{
            'port': LaunchConfiguration('port'),
            'host': LaunchConfiguration('host'),
            'topic_name': LaunchConfiguration('topic_name'),
        }],
        output='screen'
    )
    
    # Face Controller ノード
    face_controller_node = Node(
        package='face_animation_controller',
        executable='face_controller',
        name='face_controller',
        parameters=[{
            'face_api_url': LaunchConfiguration('face_api_url'),
            'topic_name': LaunchConfiguration('topic_name'),
        }],
        output='screen'
    )
    
    return LaunchDescription([
        face_api_url_arg,
        topic_name_arg,
        port_arg,
        host_arg,
        http_server_node,
        face_controller_node,
    ])