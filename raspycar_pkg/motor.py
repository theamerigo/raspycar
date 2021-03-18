import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import RPi.GPIO as gpio
import time


class MotorSubscriber(Node):

    def __init__(self, in1, in2, ena, period):
        super().__init__('motor_subscriber')
        self.subscription = self.create_subscription(
            String,
            'topic2',
            self.control_motor,
            10)
        self.subscription  # prevent unused variable warning
        self.in1 = in1
        self.in2 = in2
        self.ena = ena
        self.period = period
        gpio.setmode(gpio.BCM)
        gpio.setup(self.in1, gpio.OUT)
        gpio.setup(self.in2, gpio.OUT)
        gpio.setup(self.ena, gpio.OUT)
        gpio.output(self.in1,gpio.HIGH)
        gpio.output(self.in2,gpio.LOW)
        self.pwm = gpio.PWM(self.ena, 1000)
        self.pwm.start(0)


    def control_motor(self, msg):
        self.get_logger().info('I heard: "%s"' % msg.data)
        #data = msg.data
        #vec = data.split(':')
        duty_cycle = float(msg.data)
        self.pwm.ChangeDutyCycle(duty_cycle)
        time.sleep(self.period)



def main(args=None):
    rclpy.init(args=args)

    motor_subscriber = MotorSubscriber(17, 27, 13, 0.5)

    rclpy.spin(motor_subscriber)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    motor_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
