import RPi.GPIO as gpio
import time
import sys
import signal
import math
import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class EncoderPublisher(Node):

    def __init__(self, name, topic_pub, topic_sub, topic_log, pin, period):
        super().__init__(name)
        self.name = name
        self.publisher_ = self.create_publisher(String, topic_pub, 0)
        self.logger = self.create_publisher(String, topic_log, 0)
        self.subscription = self.create_subscription(
            String,
            topic_sub,
            self.get_target,
            0)
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
        log = String()
        tps = self.tick / self.period
        rps = tps / 20
        self.speed = 2 * math.pi * rps
        self.tick = 0
        #msg.data = 'Speed: %f' % self.speed
        #msg.data = str(self.speed)
        msg.data = str(self.speed) + ":" + str(self.target)
        log.data = self.name + " publish: " + str(self.speed) + " rad/s"
        self.publisher_.publish(msg)
        self.logger.publish(log)
        self.get_logger().info('Publishing Speed: "%s"' % msg.data)

    def encoder_signal_interrupt(self, channel):
        self.tick += 1

def signal_handler(sig, frame):
    gpio.cleanup()
    sys.exit(0)

