import rclpy
from rclpy.node import Node
from my_robot_interfaces.msg import LedStateArray
from my_robot_interfaces.srv import SetLed


class LedPanelNode(Node):
  def __init__(self):
    super().__init__('led_panel')

    self.led_states_ = [0,0,0]

    self.led_states_publisher_ = self.create_publisher(LedStateArray, 'led_states', 10)

    self.led_states_timer_ = self.create_timer(4.0, self.publish_led_states)

    self.set_led_service_ = self.create_service(SetLed, 'set_led', self.set_led_callback)

    self.get_logger().info('Led Panel Node has been started')


  def publish_led_states(self):
    msg = LedStateArray()
    msg.led_states = self.led_states_
    self.led_states_publisher_.publish(msg)

    self.get_logger().info('Publishing: ' + str(msg.led_states))

  def set_led_callback(self, request, response):
    led_number = request.led_number
    state = request.state

    if led_number < 0 or led_number >= len(self.led_states_):

      self.get_logger().warn('Wrong led number: ' + str(led_number))
      response.success = False

      return response

    if state not in [0,1]:

      self.get_logger().warn('Wrong state: ' + str(state))
      response.success = False

      return response

    self.led_states_[led_number] = state
    response.success = True
    self.publish_led_states()
    return response

def main(args=None):
    rclpy.init(args=args)
    node = LedPanelNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
