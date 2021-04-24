import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class ReferencePublisher(Node):

    def __init__(self, name, topic_R, topic_L, period, speed_R, speed_L):
        super().__init__(name)
        self.publisherR = self.create_publisher(String, topic_R, 0)
        self.publisherL = self.create_publisher(String, topic_L, 0)
        self.period = period  # seconds
        self.timer = self.create_timer(self.period, self.send_speed)
        self.i = 0
        self.speed_R = speed_R
        self.speed_L = speed_L

    def send_speed(self):
        msg_R = String()
        msg_L = String()
        if self.i < len(self.speed_L):
            msg_L.data = str(self.speed_L[self.i])
            msg_R.data = str(self.speed_R[self.i])
        else:
            msg_L.data = '0.0'
            msg_R.data = '0.0'
        self.publisherR.publish(msg_R)
        self.publisherL.publish(msg_L)
        self.get_logger().info('Publishing Reference for Left Wheel: "%s"' % msg_L.data)
        self.get_logger().info('Publishing Reference for Right Wheel: "%s"' % msg_R.data)
        self.i += 1


def main(args=None):
    rclpy.init(args=args)
    period = 0.5
    '''
    speed = [1, 2, 3, 4, 5, 6 ,7 , 8 ,9, 10, 15, 20, 25, 30, 35, 40, 45,
             45, 45, 45, 45, 45, 44, 43, 42, 41, 40, 39, 38, 37, 36, 35,
             35, 35, 35, 35, 35, 35, 35, 35, 35, 35, 35, 35, 35, 35, 35,
             35, 35, 35, 35, 35, 35, 35, 35, 35, 35, 35, 35, 35, 35, 35,
             30, 30, 30, 30, 30, 30, 30, 30, 30, 30, 30, 30, 30, 30, 30,
             0, 0, 0, 0, 10, 15, 20, 25, 25, 25, 25, 25, 25, 25, 25, 25,
             25, 20, 20, 20, 20, 20, 20, 20, 20, 20, 20, 20, 20, 20, 20,
             0, 0, 0, 0, 0]
    '''
    '''
    speed_L = [32, 32, 32, 32, 32, 32, 32, 32, 32, 32, 32, 32, 32, 32, 32,
             32, 32, 32, 32, 32, 32, 32, 32, 32, 32, 32, 32, 32, 32, 32,
             32, 32, 32, 32, 32, 32, 32, 32, 32, 32, 32, 32, 32, 32, 32,
             32, 32, 32, 32, 32, 32, 32, 32, 32, 32, 32, 32, 32, 32, 32,
             32, 32, 32, 32, 32, 32, 32, 32, 32, 32, 32, 32, 32, 32, 32]
    speed_R = [32, 32, 32, 32, 32, 32, 32, 32, 32, 32, 32, 32, 32, 32, 32,
             32, 32, 32, 32, 32, 32, 32, 32, 32, 32, 32, 32, 32, 32, 32,
             32, 32, 32, 32, 32, 32, 32, 32, 32, 32, 32, 32, 32, 32, 32,
             32, 32, 32, 32, 32, 32, 32, 32, 32, 32, 32, 32, 32, 32, 32,
             32, 32, 32, 32, 32, 32, 32, 32, 32, 32, 32, 32, 32, 32, 32]
    '''
    '''
    speed_L = [0, 0, 0, 0, 0, 28, 28, 28, 0, 28, 28, 28, 0, 28, 28, 28, 0, 28,
               28, 28, 0, 28, 28, 28, 0, 28, 28, 28, 0, 28, 28, 28, 0, 28, 28, 28, 0, 28, 28, 28, 0, 28, 28, 28, 0]
    speed_R = [0, 0, 0, 0, 0, 28, 28, 28, 28, 28, 28, 28, 28, 28, 28, 28, 28, 28, 28,
             28, 28, 28, 28, 28, 28, 28, 28, 28, 28, 28, 28, 28, 28, 28,
             28, 28, 28, 28, 28, 28, 28, 28, 28, 28, 0]
    '''
    speed_L = [0, 0, 0, 0, 0, 10, 15, 20, 25, 30, 30, 25, 20, 15, 10, 0, 0, 0, 15, 20, 15, 0, 0, 0, 10, 15, 20, 25, 30, 30, 25, 20, 15, 10, 0, 0, 0,
              15, 20, 15,  0, 0, 0, 10, 15, 20, 25, 30, 30, 25, 20, 15, 10, 0, 0, 0, 15, 20, 15,  0, 0, 0, 10, 15, 20, 25, 30, 30, 25, 20, 15, 10, 0, 0, 0]
    speed_R = [0, 0, 0, 0, 0, 10, 15, 20, 25, 30, 30, 25, 20, 15, 10, 0, 0, 0, 0, 0, 0, 0, 0, 0, 10, 15, 20, 25, 30, 30, 25, 20, 15, 10, 0, 0, 0, 0, 0,
              0, 0, 0, 0, 10, 15, 20, 25, 30, 30, 25, 20, 15, 10, 0, 0, 0, 0, 0, 0, 0, 0, 0, 10, 15, 20, 25, 30, 30, 25, 20, 15, 10, 0, 0, 0]
    node_name = "reference"
    topicR_name = "ref_to_encoderR"
    topicL_name = "ref_to_encoderL"
    reference_publisher = ReferencePublisher(node_name, topicR_name, topicL_name, period, speed_R, speed_L)

    rclpy.spin(reference_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    reference_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
