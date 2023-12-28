#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from example_interfaces.msg import Int64
from example_interfaces.srv import SetBool

class NumberCountNode(Node):

  def __init__(self):
    super().__init__('number_counter')

    self.counter_ = 0

    self.number_count_publisher_ = self.create_publisher(Int64, 'number_count',10)


    self.number_subscriber_ = self.create_subscription(Int64, 'number', self.callback_number, 10)

    self.reset_counter_service_ = self.create_service(SetBool, 'reset_counter', self.callback_reset_counter)


    self.get_logger().info('Number Counter has been started')

  def callback_reset_counter(self, request, response):
    self.counter_ = 0
    response.success = True
    response.message = 'Counter has been reset'
    return response


  def callback_number(self, msg):
    self.counter_ += msg.data

    new_msg = Int64()
    new_msg.data = self.counter_

    self.number_count_publisher_.publish(new_msg)

    self.get_logger().info(str(self.counter_))

def main(args=None):

  rclpy.init(args=args)

  node = NumberCountNode()

  rclpy.spin(node) # spin() will block until the node is shutdown
  rclpy.shutdown()


if __name__ == '__main__':
  main()