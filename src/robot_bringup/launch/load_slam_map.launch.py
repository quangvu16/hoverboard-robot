from launch import LaunchDescription
from launch.actions import OpaqueFunction
from launch_ros.actions import Node
import rclpy
from lifecycle_msgs.srv import GetState

def check_lifecycle_node_active(context, *args, **kwargs):
    rclpy.init()
    node = rclpy.create_node('wait_for_lifecycle')
    client = node.create_client(GetState, '/slam_toolbox/get_state')

    while not client.wait_for_service(timeout_sec=1.0):
        node.get_logger().info('Waiting for /slam_toolbox/get_state service...')

    req = GetState.Request()
    while rclpy.ok():
        future = client.call_async(req)
        rclpy.spin_until_future_complete(node, future)
        if future.result() and future.result().current_state.label == 'active':
            node.get_logger().info('slam_toolbox is ACTIVE. Proceeding.')
            break
        node.get_logger().info('Waiting for slam_toolbox to become active...')

    node.destroy_node()
    rclpy.shutdown()

    return [
        Node(
            package='robot_bringup',
            executable='load_slam_map.py',
            name='load_slam_map',
            output='screen'
        )
    ]

def generate_launch_description():
    return LaunchDescription([
        OpaqueFunction(function=check_lifecycle_node_active)
    ])
