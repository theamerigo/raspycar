import rclpy
from .motor import MotorSubscriber


def main(args=None):
    rclpy.init(args=args)
    name = "right_motor"
    topic = "pwm_to_motorR"
    in1 = 2
    in2 = 3
    ena = 4
    period = 0.25
    motor_subscriber = MotorSubscriber(name, topic, in1, in2, ena, period)

    rclpy.spin(motor_subscriber)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    motor_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

