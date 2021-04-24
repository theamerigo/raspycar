import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class PIDSubscriber(Node):

    def __init__(self, name, topic_pub, topic_sub, P, I, D, min, max, period):
        super().__init__(name)
        self.subscription = self.create_subscription(
            String,
            topic_sub,
            self.pid_control,
            0)
        self.publisher = self.create_publisher(String, topic_pub, 0)
        self.subscription  # prevent unused variable warning
        self.period = period
        self.P = P
        self.I = I
        self.D = D
        self.proportional = 0
        self.integral = 0
        self.derivative = 0
        self.min = min
        self.max = max
        self.control = 0
        self.timer = self.create_timer(self.period, self.send_control)

    def pid_control(self, msg):
        self.get_logger().info('I heard: "%s"' % msg.data)
        data = msg.data
        vec = data.split(':')
        speed = float(vec[0])
        target = float(vec[1])
        if target == 0.0:
            self.integral = 0
        error = target - speed
        self.integral += error
        self.control = self.P * error + self.I * self.integral
        if self.control > self.max:
            self.control = self.max
        elif self.control < self.min:
            self.control = self.min
        #self.get_logger().info('I calculate pid control = "%f"' % self.control)

    def send_control(self):
        msg = String()
        msg.data = str(self.control)
        self.publisher.publish(msg)
        self.get_logger().info('Publishing Duty Cycle: "%s"' % msg.data)

def main(args=None):
    rclpy.init(args=args)
    name = "pid"
    topic_pub = "pwm_to_motor"
    topic_sub = "speed_to_pid"
    P = 1
    I = 1
    D = 0
    min = 0
    max = 100
    period = 0.5
    pid_subscriber = PIDSubscriber(name, topic_pub, topic_sub, P, I, D, min, max, period)

    rclpy.spin(pid_subscriber)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    pid_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
