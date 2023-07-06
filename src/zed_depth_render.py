import rclpy
import cv2
from cv_bridge import CvBridge, CvBridgeError
from rclpy.node import Node
from sensor_msgs.msg import Image

class DepthSubscriber(Node):
    def __init__(self):
        super().__init__('depth_subscriber')
        self.subscription = self.create_subscription(
            Image,
            '/zed2/zed_node/depth/depth_registered',
            self.callback,
            10
        )
        self.subscription  # Prevent unused variable warning

        self._bridge = CvBridge()


    def callback(self, msg):
        # 처리할 코드를 작성합니다.
        # 수신한 이미지 메시지를 활용하여 원하는 작업을 수행합니다.
        # 예시로서, 이미지의 너비와 높이를 출력합니다.
        self.get_logger().info('Received depth image with width: %d, height: %d' % (msg.width, msg.height))
        image = self._bridge.imgmsg_to_cv2(msg, "32FC1")
        cv2.imshow('image', image)
        cv2.waitKey(3)


def main(args=None):
    rclpy.init(args=args)
    depth_subscriber = DepthSubscriber()
    rclpy.spin(depth_subscriber)
    depth_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()