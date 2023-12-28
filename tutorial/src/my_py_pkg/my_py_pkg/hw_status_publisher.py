import rclpy
from rclpy.node import Node
from my_robot_interfaces.msg import HardwareStatus

class HwStatusPublisher(Node):
  def __init__(self):
    super().__init__('hw_status_publisher')

    self.hw_status_publisher_ = self.create_publisher(HardwareStatus, 'hardware_status', 10)

    self.timer_ = self.create_timer(1.0, self.publish_hw_status)

    self.get_logger().info('HwStatusPublisher node has been initialized')

  def publish_hw_status(self):
    msg = HardwareStatus()
    msg.temperature = 45
    msg.are_motors_ready = True
    msg.debug_message = 'Everything is OK'

    self.hw_status_publisher_.publish(msg)

def main(args=None):
  rclpy.init(args=args)
  hw_status_publisher = HwStatusPublisher()
  rclpy.spin(hw_status_publisher)
  hw_status_publisher.destroy_node()
  rclpy.shutdown()

if __name__ == '__main__':
    main()
