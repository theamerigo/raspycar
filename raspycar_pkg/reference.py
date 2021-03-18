import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class ReferencePublisher(Node):

    def __init__(self, period, speed):
        super().__init__('reference_publisher')
        self.publisher_ = self.create_publisher(String, 'topic3', 10)
        self.period = period  # seconds
        self.timer = self.create_timer(self.period, self.send_speed)
        self.i = 0
        self.speed = speed

    def send_speed(self):
        msg = String()
        if self.i < len(self.speed):
            msg.data = str(self.speed[self.i])
        else:
            msg.data = '0.0'
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing Reference: "%s"' % msg.data)
        self.i += 1


def main(args=None):
    rclpy.init(args=args)
    period = 0.5
    speed = [1, 2, 3, 4, 5, 6 ,7 , 8 ,9, 10, 15, 20, 25, 30, 35, 40, 45,
             45, 45, 45, 45, 45, 44, 43, 42, 41, 40, 39, 38, 37, 36, 35,
             35, 35, 35, 35, 35, 35, 35, 35, 35, 35, 35, 35, 35, 35, 35,
             30, 30, 30, 30, 30, 30, 30, 30, 25, 25, 25, 25, 25, 25, 25,
             20, 20, 20, 20, 20, 15, 15, 15, 15, 15, 15, 15, 15, 0, 0, 0]
    reference_publisher = ReferencePublisher(period, speed)

    rclpy.spin(reference_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    reference_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
