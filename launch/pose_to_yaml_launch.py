from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    return LaunchDescription([
        
        DeclareLaunchArgument('topic_name', default_value='/localization/pose_twist_fusion_filter/pose', description='Name of the topic to subscribe to'),
        DeclareLaunchArgument('output_dir', default_value='./', description='Directory where YAML files will be saved'),
        DeclareLaunchArgument('first_pose_yaml', default_value='', description='Path to the YAML file containing the first pose in map frame'),

        Node(
            package='pose_to_yaml',
            executable='pose_to_yaml_node',
            name='pose_to_yaml_node',
            output='screen',
            parameters=[{
                'output_dir': LaunchConfiguration('output_dir'),
                'topic_name': LaunchConfiguration('topic_name'),
                'first_pose_yaml': LaunchConfiguration('first_pose_yaml')
            }]
        )
    ])
