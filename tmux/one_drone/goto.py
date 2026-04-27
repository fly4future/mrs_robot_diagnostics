#!/usr/bin/python3

import os
import sys
import rclpy
from rclpy.node import Node
from mrs_msgs.srv import Vec4

class Goto(Node):

    def __init__(self, uav_name):
        super().__init__(f'{uav_name}_goto')

        # self.get_logger().info('ROS2 node initialized')
        self.get_logger().info(f'Setting up the Goto service client for {uav_name}')

        self.service_topic = f"/{uav_name}/control_manager/goto"
        self.client = self.create_client(Vec4, self.service_topic)

        self.timer = self.create_timer(0.1, self.doAction)
        # self.get_logger().info('__init__ finished')

    def doAction(self):
        # self.get_logger().info('doing the action')
        self.timer.cancel()

        while not self.client.wait_for_service(timeout_sec=3.0):
            self.get_logger().warn(f"Service '{self.service_topic}' not available, waiting again...")

        request = Vec4.Request()
        request.goal[0] = 5.0
        request.goal[1] = 5.0
        request.goal[2] = 2.0
        request.goal[3] = 1.5

        self.get_logger().info('Calling service')

        future = self.client.call_async(request)
        future.add_done_callback(self.doneCallback)

    def doneCallback(self, future):
        try:
            response = future.result()
            self.get_logger().info(f"Response: {response}")
        except Exception as e:
            self.get_logger().error(f"Service call failed: {e}")

        rclpy.shutdown()

def main(args=None):

    uav_name = os.environ.get('UAV_NAME')
    if not uav_name:
        print("ERROR: UAV_NAME environment variable is not set!")
        print("Usage example: UAV_NAME=uav1 ./goto.py")
        sys.exit(1)

    rclpy.init(args=args)
    node = Goto(uav_name)

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
