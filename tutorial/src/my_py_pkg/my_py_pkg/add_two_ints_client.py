#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from example_interfaces.srv import AddTwoInts
from functools import partial


class AddTwoIntsClient(Node):
  def __init__(self):
    super().__init__('add_two_ints_client')
    self.call_add_two_ints(1,2)


  def call_add_two_ints(self,a,b):
    self.client = self.create_client(AddTwoInts, 'add_two_ints')

    while not self.client.wait_for_service(timeout_sec=1.0):
      self.get_logger().warn('service not available, waiting again...')


    req = AddTwoInts.Request()
    req.a = a
    req.b = b

    future = self.client.call_async(req)

    # i can not pass the arguments to the callback function
    # so i use partial to pass the arguments

    future.add_done_callback(partial(self.callback_call_add_two_ints,a=a,b=b))

  def callback_call_add_two_ints(self, future,a,b):

    try:
      response = future.result()
      self.get_logger().info('Result of add_two_ints: %d' % (response.sum))

    except Exception as e:
      self.get_logger().error('Service call failed %r' % (e,))



def main(args=None):
  rclpy.init(args=args)

  node = AddTwoIntsClient()
  rclpy.spin(node)

  rclpy.shutdown()


if __name__ == '__main__':
  main()