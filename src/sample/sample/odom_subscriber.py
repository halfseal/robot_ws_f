import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
import time

class OdomSubscriber(Node):

    def __init__(self):
        super().__init__('odom_subscriber')
        self.subscription = self.create_subscription(
            Odometry,
            '/odom',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

        self.timer_period = 1  # seconds
        self.timer = self.create_timer(self.timer_period, self.timer_callback)
        self.msg = None

    def listener_callback(self, msg):
        x = float(msg.pose.pose.position.x)
        y = float(msg.pose.pose.position.y)
        z = float(msg.pose.pose.position.z)
        self.msg = 'Robot position: x=%.3f, y=%.3f, z=%.3f' % (x, y, z)

    def timer_callback(self):
        if self.msg is not None:
            self.get_logger().info(self.msg)


def main(args=None):
    rclpy.init(args=args)

    odom_subscriber = OdomSubscriber()

    rclpy.spin(odom_subscriber)

    # Destroy the node explicitly
    odom_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
