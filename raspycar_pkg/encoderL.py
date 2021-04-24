from .encoder import EncoderPublisher
import rclpy


def main(args=None):
    encoder_pin = 22
    period = 0.5
    rclpy.init(args=args)
    name = 'left_encoder'
    topic_sub = 'ref_to_encoderL'
    topic_pub = 'speed_to_pidL'
    topic_log = 'encoderL_logger'
    encoder_publisher = EncoderPublisher(name, topic_pub, topic_sub, topic_log, encoder_pin, period)
    rclpy.spin(encoder_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    encoder_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

