import time
import sys
import signal
import math
import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class OdometryPublisher(Node):

    def __init__(self, name, topic_pub, topic_sub, topic_log, wheel_radius, wheels_distance, period):
        super().__init__(name)
        self.name = name
        self.publisher_ = self.create_publisher(String, topic_pub, 0)
        self.logger = self.create_publisher(String, topic_log, 0)
        self.subscriptionL = self.create_subscription(
            String,
            topic_sub[0],
            self.get_encoderL,
            0)
        self.subscriptionR = self.create_subscription(
            String,
            topic_sub[1],
            self.get_encoderR,
            0)
        self.period = period   # seconds
        self.omegaL = 0
        self.omegaR = 0
        self.r = wheel_radius
        self.l = wheels_distance
        self.timer = self.create_timer(self.period, self.send_odometry)

    def get_encoderL(self, msg):
        self.get_logger().info('I heard: "%s"' % msg.data)
        data = msg.data
        vec = data.split(":")
        self.omegaL = float(vec[0])

    def get_encoderR(self, msg):
        self.get_logger().info('I heard: "%s"' % msg.data)
        data = msg.data
        vec = data.split(":")
        self.omegaR = float(vec[0])

    def send_odometry(self):
        msg = String()
        log = String()
        theta_dot = self.r / self.l * (self.omegaR - self.omegaL)
        theta = theta_dot * self.period
        x_dot = self.r / 2 * (self.omegaR + self.omegaL) * math.cos(theta)
        x = x_dot * self.period
        y_dot = self.r / 2 * (self.omegaR + self.omegaL) * math.sin(theta)
        y = y_dot * self.period
        msg.data = str(x) + ":" + str(y) + ":" + str(theta)
        log.data = self.name + " publish: " + msg.data
        self.publisher_.publish(msg)
        self.logger.publish(log)
        self.get_logger().info('Publishing Odometry: "%s"' % msg.data)


def main(args=None):
    rclpy.init(args=args)
    name = "odometry_node"
    topic_pub = "path_plannning_odometry"
    topic_sub = ["speed_to_pidL", "speed_to_pidR"]
    topic_log = "odometry_logger"
    wheel_radius = 0.05 #m
    wheels_distance = 0.10 #m
    period = 0.5
    odometry = OdometryPublisher(name, topic_pub, topic_sub, topic_log, wheel_radius, wheels_distance, period)

    rclpy.spin(odometry)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    odometry.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
