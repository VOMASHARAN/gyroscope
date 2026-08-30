from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    nodes = []

    for gyro_id in range(1, 9):
        nodes.append(
            Node(
                package='multi_gyro_fusion',
                executable='gyro_publisher',
                name=f'gyro_{gyro_id}',
                output='screen',
                parameters=[
                    {
                        'gyro_id': gyro_id,
                        'noise_stddev': 0.01,
                        'bias_x': 0.0,
                        'bias_y': 0.0,
                        'bias_z': 0.0,
                        'frequency': 100.0,
                        'arw': 0.01,
                        'rrw': 0.001,
                        'markov_correlation': 0.5,
                        'markov_tau': 1.0,
                        'markov_stddev': 0.01,
                    }
                ],
            )
        )

    return LaunchDescription(nodes)
