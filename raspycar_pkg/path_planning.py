import time
import sys
import signal
import math
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import numpy as np
from numpy.linalg import inv


class PathPlanningPublisher(Node):

    def __init__(self, name, topic_pub, topic_sub, topic_log, wheel_radious, wheels_distance, initial_position, final_position, period):
        super().__init__(name)
        self.name = name
        self.publisherL = self.create_publisher(String, topic_pub[0], 0)
        self.publisherR = self.create_publisher(String, topic_pub[1], 0)
        self.logger = self.create_publisher(String, topic_log, 0)
        self.subscription = self.create_subscription(
            String,
            topic_sub,
            self.get_odometry,
            0)
        self.period = period   # seconds
        self.omegaL = 0
        self.omegaR = 0
        self.r = wheel_radious
        self.l = wheels_distance
        self.J = np.array([[self.r/2, self.r/2],[-self.r/self.l, self.r/self.l]])
        self.J_inv = inv(self.J)
        self.initial_position = initial_position
        self.final_position = final_position
        self.x_t = initial_position[0]
        self.y_t = initial_position[1]
        self.theta_t = initial_position[2]
        self.x_d = final_position[0]
        self.y_d = final_position[1]
        self.theta_d = np.arctan2(self.y_d - self.y_t, self.x_d - self.x_t)
        self.x_odometry = 0
        self.y_odometry = 0
        self.theta_odometry = 0
        self.k_theta = 1 #da tarare
        self.k_v = 1 #da tarare
        self.timer = self.create_timer(self.period, self.send_velocity)

    def get_odometry(self, msg):
        self.get_logger().info('I heard: "%s"' % msg.data)
        data = msg.data
        vec = data.split(":")
        self.x_odometry = float(vec[0])
        self.y_odometry = float(vec[1])
        self.theta_odometry = float(vec[2])

    def send_velocity(self):
        msgL = String()
        msgR = String()
        log = String()
        self.x_t += self.x_odometry
        self.y_t += self.y_odometry
        self.theta_t += self.theta_odometry
        theta_dot = self.k_theta * (self.theta_d - self.theta_t)
        v_b = math.sqrt((self.x_d - self.x_t) ** 2 + (self.y_d - self.y_t) ** 2)
        res = np.dot(self.J_inv, np.array([v_b, theta_dot]))
        self.omegaR = res[0]
        self.omegaL = res[1]
        msgR.data = str(self.omegaR)
        msgL.data = str(self.omegaL)
        log.data = self.name + " publish: omegaR = " + msgR.data + " rad/r - omegaL = " + msgL.data + " rad/s"
        self.publisherR.publish(msgR)
        self.publisherL.publish(msgL)
        self.logger.publish(log)
        self.get_logger().info('Publishing angular velocities: "%s"' % log.data)


def main(args=None):
    rclpy.init(args=args)
    name = "path_planning_node"
    topic_pub = ["ref_to_encoderL", "ref_to_encoderR"]
    topic_sub = "path_plannning_odometry"
    topic_log = "path_planning_logger"
    wheel_radius = 0.05 #m
    wheels_distance = 0.10 #m
    period = 0.5
    initial_position = [0, 0, 0]
    final_position = [0.1, 0, 0]
    path_planning = PathPlanningPublisher(name, topic_pub, topic_sub, topic_log, wheel_radius, wheels_distance, initial_position, final_position, period)

    rclpy.spin(path_planning)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    path_planning.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
