import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

from nav2_msgs.action import FollowWaypoints
from geometry_msgs.msg import PoseStamped


import numpy as np




def create_pose(x, y):
    pose = PoseStamped()
    pose.header.frame_id = 'map'
    pose.pose.position.x = x
    pose.pose.position.y = y
    return pose



def create_line(x, y):
    (a1, b1) = x
    (a2, b2) = y
    return [create_pose(a2 * p + a1 * (1 - p), b2 *p + b1 * (1 - p)) for p in np.arange(0, 1, 0.1)]




class FollowActionClient(Node):

    def __init__(self):
        super().__init__('waypoint_action_client')
        self._action_client = ActionClient(self, FollowWaypoints, '/otto_1/follow_waypoints')

    def send_goal(self, x, y):
        goal_msg = FollowWaypoints.Goal()

        # goal_msg.poses = [create_pose(x, 1.23) for x in [-9.0, -8.0, -7.0, -6.0, -5.0, -4.0, -3.0, -2.0, -1.0, 0.0, 1.0]]


        if(len(self.goals) < 2):
            rclpy.shutdown()
            return True

        x = self.goals[0]
        y = self.goals[1]
        self.goals = self.goals[1:]

        goal_msg.poses = create_line(x, y)

        self._action_client.wait_for_server()

        self._send_goal_future = self._action_client.send_goal_async(goal_msg, feedback_callback=self.feedback_callback)

        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected :(')
            return

        self.get_logger().info('Goal accepted :)')

        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info('Result: {0}'.format(result.missed_waypoints))
        self.send_goal(0, 0)
        # rclpy.shutdown()

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.get_logger().info('Received feedback: {0}'.format(feedback.current_waypoint))




def main(args=None):
    rclpy.init(args=args)

    action_client = FollowActionClient()


    action_client.goals = [(-9.0, 1.23), (3.0, 1.23), (-9.0, 1.23)]

    action_client.send_goal((-9.0, 1.23), (3.0, 1.23))

    # action_client.send_goal((3.0, 1.23), (-9.0, 1.23))


    rclpy.spin(action_client)


if __name__ == '__main__':
    main()