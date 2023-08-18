import rclpy
from rclpy.action import ActionClient
from ur_moveit_demo_msg.action import FollowPath  

ns = "base_link"
def follow_path_client():
    rclpy.init()

    node = rclpy.create_node('follow_path_client')

    # Create an action client
    action_client = ActionClient(node, FollowPath, ns+"/follow_path")

    # Wait for the action server to be available
    if not action_client.wait_for_server(timeout_sec=10.0):
        node.get_logger().error('Action server not available')
        return

    # Create a goal
    goal_msg = FollowPath.Goal()

    # Send the goal
    send_goal_future = action_client.send_goal_async(goal_msg)

    # Wait for the goal to complete
    rclpy.spin_until_future_complete(node, send_goal_future)
    goal_handle = send_goal_future.result()

    if not goal_handle.accepted:
        node.get_logger().info('Goal rejected')
        return

    # Wait for the result
    get_result_future = goal_handle.get_result_async()

    rclpy.spin_until_future_complete(node, get_result_future)
    result = get_result_future.result()

    if result:
        node.get_logger().info('Received result: %r' % str(result))
    else:
        node.get_logger().info('Result not available')

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    follow_path_client()
