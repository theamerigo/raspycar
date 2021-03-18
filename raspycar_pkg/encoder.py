import RPi.GPIO as gpio
import time
import sys
import signal
import math
import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class EncoderPublisher(Node):

    def __init__(self, pin, period):
        super().__init__('encoder_publisher')
        self.publisher_ = self.create_publisher(String, 'topic1', 10)
        self.subscription = self.create_subscription(
            String,
            'topic3',
            self.get_target,
            10)
        self.period = period   # seconds
        self.timer = self.create_timer(self.period, self.get_speed)
        self.pin = pin
        self.tick = 0
        self.speed = 0.0
        self.target = 0.0
        gpio.setmode(gpio.BCM)
        print('Setup Interrupt on gpio ' + str(self.pin))
        gpio.setup(self.pin, gpio.IN, pull_up_down=gpio.PUD_UP)
        gpio.add_event_detect(self.pin, gpio.RISING)
        gpio.add_event_callback(self.pin, self.encoder_signal_interrupt)

    def get_target(self, msg):
        self.get_logger().info('I heard: "%s"' % msg.data)
        self.target = float(msg.data)

    def get_speed(self):
        msg = String()
        tps = self.tick / self.period
        rps = tps / 20
        self.speed = 2 * math.pi * rps
        self.tick = 0
        #msg.data = 'Speed: %f' % self.speed
        #msg.data = str(self.speed)
        msg.data = str(self.speed) + ":" + str(self.target)
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing Speed: "%s"' % msg.data)

    def encoder_signal_interrupt(self, channel):
        self.tick += 1

def signal_handler(sig, frame):
    gpio.cleanup()
    sys.exit(0)

def main(args=None):
    encoder_pin = 22
    period = 0.5
    rclpy.init(args=args)
    encoder_publisher = EncoderPublisher(encoder_pin, period)
    rclpy.spin(encoder_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    encoder_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
