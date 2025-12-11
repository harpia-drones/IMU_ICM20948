from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description()

    # NODE: Publish raw IMU data (accel and gyro)
    raw_data_publisher_node = Node(
        package='imu_bringup',
        executable='imu_raw_data_publisher.py',
        name='imu_raw_data_publisher',
        output=screen
    )

    # Publish IMU data with orientation
    orientation_publisher_node = Node(
        package='imu_filter_madgwick',
        executable='imu_filter_madgwick_node',
        name='imu_publisher',
        parameters=[
            {'use_mag': False},
            {'use_sim_time': False}
        ],
        output=screen
    )

    return LaunchDescription([
        raw_data_publisher_node,
        orientation_publisher_node
    ])