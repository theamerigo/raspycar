from .encoder import EncoderPublisher
import rclpy


def main(args=None):
    encoder_pin = 10
    period = 0.5
    rclpy.init(args=args)
    name = 'right_encoder'
    topic_sub = 'ref_to_encoderR'
    topic_pub = 'speed_to_pidR'
    encoder_publisher = EncoderPublisher(name, topic_pub, topic_sub, encoder_pin, period)
    rclpy.spin(encoder_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    encoder_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

