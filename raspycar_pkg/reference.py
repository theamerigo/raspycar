import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class ReferencePublisher(Node):

    def __init__(self, period, speed):
        super().__init__('reference_publisher')
        self.publisher_ = self.create_publisher(String, 'topic3', 10)
        self.period = period  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0
        self.speed = speed

    def send_speed(self):
        msg = String()
        msg.data = 'Speed %f' % self.speed[i]
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.data)
        self.i += 1


def main(args=None):
    rclpy.init(args=args)

    reference_publisher = ReferencePublisher()

    rclpy.spin(reference_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    reference_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
