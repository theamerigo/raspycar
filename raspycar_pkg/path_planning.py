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

    def __init__(self, name, topic_pub, topic_sub, topic_log, k_v, k_theta, wheel_radious, wheels_distance, omegaR_max, omegaL_max, initial_position, final_position, period):
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
        self.omegaR_max = omegaR_max
        self.omegaL_max = omegaL_max
        self.J = np.array([[self.r/2, self.r/2],[-self.r/self.l, self.r/self.l]])
        self.J_inv = inv(self.J)
        self.initial_position = initial_position
        self.final_position = final_position
        self.x_t = initial_position[0]
        self.y_t = initial_position[1]
        self.theta_t = initial_position[2]
        #self.theta_t = 0
        self.x_d = final_position[0]
        self.y_d = final_position[1]
        self.theta_d = np.arctan2(self.y_d - self.y_t, self.x_d - self.x_t)
        self.x_odometry = 0
        self.y_odometry = 0
        self.theta_odometry = 0
        self.v_b = 0
        self.v_b_pre = 0
        self.k_theta = k_theta
        self.k_v = k_v
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
        log_position = String()
        self.x_t += self.x_odometry
        self.y_t += self.y_odometry
        self.theta_t = self.theta_odometry
        theta_dot = self.k_theta * (self.theta_d - self.theta_t)
        self.v_b_pre = self.v_b
        self.v_b = math.sqrt((self.x_d - self.x_t) ** 2 + (self.y_d - self.y_t) ** 2)
        self.get_logger().info('Path planning publish new v_b = "%s"' % str(self.v_b))
        if self.v_b < 0.25 or self.v_b_pre < self.v_b:
            self.omegaR = 0
            self.omegaL = 0
        else:
            res = np.dot(self.J_inv, np.array([self.v_b, theta_dot]))
            self.omegaL = res[0]
            self.omegaR = res[1]
            diff = abs(self.omegaR - self.omegaL)
            if self.omegaR > self.omegaR_max and self.omegaL > self.omegaL_max:
                if self.omegaR > self.omegaL:
                    self.omegaR = self.omegaR_max
                    self.omegaL = self.omegaL_max - diff
                else:
                    self.omegaL = self.omegaL_max
                    self.omegaR = self.omegaR_max - diff
            elif self.omegaR > self.omegaR_max and self.omegaL <= self.omegaL_max:
                diff = abs(self.omegaR - self.omegaR_max)
                self.omegaR = self.omegaR_max
                self.omegaL -= diff
            elif self.omegaR <= self.omegaR_max and self.omegaL > self.omegaL_max:
                diff = abs(self.omegaL - self.omegaL_max)
                self.omegaL = self.omegaL_max
                self.omegaR -= diff
        msgR.data = str(self.omegaR)
        msgL.data = str(self.omegaL)
        log.data = self.name + " publish: omegaR = " + msgR.data + " rad/r - omegaL = " + msgL.data + " rad/s"
        log_position.data = self.name + "publish x_t = " + str(self.x_t) + " y_t = " + str(self.y_t) + " theta_t = " + str(self.theta_t)
        self.publisherR.publish(msgR)
        self.publisherL.publish(msgL)
        self.logger.publish(log)
        self.get_logger().info('Publishing angular velocities: "%s"' % log.data)
        self.get_logger().info('Publishing current position: "%s"' % log_position.data)


def main(args=None):
    rclpy.init(args=args)
    name = "path_planning_node"
    topic_pub = ["ref_to_encoderL", "ref_to_encoderR"]
    topic_sub = "path_plannning_odometry"
    topic_log = "path_planning_logger"
    wheel_radius = 0.033 #m
    wheels_distance = 0.12 #m
    period = 0.25
    initial_position = [0, 0, math.pi/2]
    final_position = [0, 1, 0]
    k_v = 1
    k_theta = 1
    omegaR_max = 12 #rad/s
    omegaL_max = 12 #rad/s
    path_planning = PathPlanningPublisher(name, topic_pub, topic_sub, topic_log, k_v, k_theta, wheel_radius, wheels_distance, omegaR_max, omegaL_max, initial_position, final_position, period)
    print(final_position)
    rclpy.spin(path_planning)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    path_planning.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
