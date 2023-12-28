import rclpy
from rclpy.node import Node
from functools import partial

from my_robot_interfaces.srv import SetLed

class BatteryNode(Node):
  def __init__(self):
    super().__init__('battery_node')

    self.batter_state_ = "full"

    self.last_time_battery_state_ = self.get_current_time()

    self.battery_timer_ = self.create_timer(0.1, self.check_battery_state)


    self.get_logger().info('Battery node initialized')


  def check_battery_state(self):

    timer_now = self.get_current_time()

    if self.batter_state_ == "full":
      if timer_now - self.last_time_battery_state_ > 4.0:
        self.batter_state_ = "empty"
        self.call_set_led_server(2,1)
        self.get_logger().info('Battery empty')
        self.last_time_battery_state_ = timer_now
    else:
      if timer_now - self.last_time_battery_state_ > 6.0:
        self.batter_state_ = "full"
        self.call_set_led_server(2,0)
        self.get_logger().info('Battery full')
        self.last_time_battery_state_ = timer_now


  def call_set_led_server(self, led_number, state):
    client = self.create_client(SetLed, 'set_led')
    while not client.wait_for_service(1.0):
      self.get_logger().warn('Waiting for Server SetLed...')

    request = SetLed.Request()

    request.led_number = led_number
    request.state = state

    future = client.call_async(request)
    future.add_done_callback(partial(self.callback_call_set_led, led_number = led_number, state = state))


  def callback_call_set_led(self, future, led_number, state):
    try:
      response = future.result()
      self.get_logger().info(str(response.success))

    except Exception as e:
      self.get_logger().error('Service call failed %r' % (e,))


  def get_current_time(self):
    secs, nsecs =  self.get_clock().now().seconds_nanoseconds()

    return secs + nsecs/1000000000.0

def main(args=None):
  rclpy.init(args=args)
  node = BatteryNode()
  rclpy.spin(node)
  rclpy.shutdown()

if __name__ == '__main__':
    main()
