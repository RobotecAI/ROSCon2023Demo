# Copyright (c) Contributors to the Open 3D Engine Project.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

from gazebo_msgs.srv import SpawnEntity

import rclpy
from rclpy.exceptions import ParameterUninitializedException
from rclpy.node import Node


class RobotSpawner(Node):

    def __init__(self):
        super().__init__('robot_spawner_client')

        self.declare_parameter('robot_name', rclpy.Parameter.Type.STRING)
        self.declare_parameter('robot_namespace', '')
        self.declare_parameter('robot_initial_position', [0.0, 0.0, 0.0])

        try:
            self.robot_name = self.get_parameter('robot_name').get_parameter_value().string_value
        except ParameterUninitializedException as e:
            self.get_logger().warn("Nothing to spawn. 'robot_name' parameter not set.")
            return
        
        self.robot_namespace = self.get_parameter('robot_namespace').get_parameter_value().string_value
        self.robot_initial_position = self.get_parameter('robot_initial_position').get_parameter_value().double_array_value


        self.cli = self.create_client(SpawnEntity, 'spawn_entity')
        
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Spawning service not available, waiting...')

        self.req = SpawnEntity.Request()
        result = self.send_request()

        if result.success:
            self.get_logger().info(f'{self.robot_namespace} spawned.')
        else:
            self.get_logger().error(f'Failed to spawn {self.robot_namespace}: {result.status_message}')

    def send_request(self):
        self.get_logger().info(f"Spawning robot: {self.robot_namespace}/{self.robot_name} on ("
                            f"{self.robot_initial_position[0]}, "
                            f"{self.robot_initial_position[1]}, "
                            f"{self.robot_initial_position[2]})")
        
        self.req.name = self.robot_name
        self.req.robot_namespace = self.robot_namespace
        self.req.initial_pose.position.x = self.robot_initial_position[0]
        self.req.initial_pose.position.y = self.robot_initial_position[1]
        self.req.initial_pose.position.z = self.robot_initial_position[2]

        self.future = self.cli.call_async(self.req)
        rclpy.spin_until_future_complete(self, self.future)

        return self.future.result()


def main(args=None):
    rclpy.init(args=args)

    robot_spawner = RobotSpawner()
    robot_spawner.destroy_node()

    rclpy.shutdown()

if __name__ == '__main__':
    main()