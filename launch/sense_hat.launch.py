from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='ros2_pi_sense_hat',
            executable='led_matrix_node',
            name='led_matrix_node',
            output='screen'
        ),
        Node(
            package='ros2_pi_sense_hat',
            executable='joystick_node',
            name='joystick_node',
            output='screen'
        ),
        Node(
            package='ros2_pi_sense_hat',
            executable='imu_node',
            name='imu_node',
            output='screen'
        ),
        Node(
            package='ros2_pi_sense_hat',
            executable='environmental_node',
            name='environmental_node',
            output='screen',
            parameters=[{
                'publish_rate': 1,
                'temperature_offset_hts221': 0.0,
                'temperature_offset_lps25h': 0.0,
                'hts221_odr': 1,
                'lps25h_odr': 1
            }]
        ),
        Node(
            package='ros2_pi_sense_hat',
            executable='color_node',
            name='color_node',
            output='screen',
            parameters=[{
                'publish_rate': 10,
                'integration_time': 0xF6,
                'gain': 2,
                'lux_calibration': 1.0
            }]
        ),
    ])
