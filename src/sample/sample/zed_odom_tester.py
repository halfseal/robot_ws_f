import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry

class OdomListener(Node):
    def __init__(self):
        super().__init__('odom_listener')
        self.subscription = self.create_subscription(
            Odometry,
            'zed2/zed_node/odom',
            self.odom_callback,
            10
        )
        self.subscription  # To prevent unused variable warning

    def odom_callback(self, msg):
        # 메시지 수신시 실행되는 콜백 함수
        # 수신된 메시지의 내용을 원하는 방식으로 처리하면 됩니다.
        self.get_logger().info(f"Received Odometry: {msg}")

def main(args=None):
    rclpy.init(args=args)
    odom_listener = OdomListener()
    rclpy.spin(odom_listener)
    odom_listener.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
