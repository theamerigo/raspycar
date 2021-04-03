import rclpy
from .motor import MotorSubscriber


def main(args=None):
    rclpy.init(args=args)
    name = "motor"
    topic = "pwm_to_motor"
    in1 = 17
    in2 = 27
    ena = 13
    period = 0.5
    motor_subscriber = MotorSubscriber(name, topic, in1, in2, ena, period)

    rclpy.spin(motor_subscriber)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    motor_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

