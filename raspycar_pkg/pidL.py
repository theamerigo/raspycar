import rclpy
from .pid import PIDSubscriber


def main(args=None):
    rclpy.init(args=args)
    name = "left_pid_control"
    topic_pub = "pwm_to_motorL"
    topic_sub = "speed_to_pidL"
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
