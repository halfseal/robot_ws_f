import rclpy
import cv2
import numpy as np
from sensor_msgs.msg import Image, PointCloud, ChannelFloat32
from geometry_msgs.msg import Point32
from cv_bridge import CvBridge, CvBridgeError
from rclpy.node import Node


class DepthSubscriber(Node):
    def __init__(self):
        super().__init__('depth_subscriber')
        self.subscription = self.create_subscription(
            Image,
            '/zed2/zed_node/depth/depth_registered',
            self.callback,
            10
        )
        # self.subscription  # Prevent unused variable warning

        self.publisher = self.create_publisher(PointCloud, '/sp/pointcloud', 10)
        self.bridge = CvBridge()

    def callback(self, msg):
        try:
            # self.get_logger().info('Received depth image with width: %d, height: %d' % (msg.width, msg.height))

            image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
            # cv2.imshow("Image", image)
            # cv2.waitKey(1)  # Display the image for 1 millisecond

            depth_image = np.array(image, dtype=np.float32)

            # Convert depth image to point cloud
            height, width = depth_image.shape
            points = []
            for v in range(height):
                for u in range(width):
                    depth = depth_image[v, u]
                    if depth > 0.0:  # Exclude invalid depths
                        point = self.convert_depth_to_point(u, v, depth)
                        points.append(point)
                        pass

            # Create PointCloud message
            pc_msg = PointCloud()
            pc_msg.header = msg.header

            # Convert points to Point32 messages
            point32_msgs = [Point32(x=point[0], y=point[1], z=point[2]) for point in points]
            pc_msg.points = point32_msgs

            # Add intensity channel (optional)
            intensity_channel = ChannelFloat32()
            intensity_channel.name = 'intensity'
            intensity_channel.values = [1.0] * len(points)
            pc_msg.channels.append(intensity_channel)

            # Publish the PointCloud message
            self.publisher.publish(pc_msg)
            print("Published point cloud with {} points".format(len(points)))

        except CvBridgeError as e:
            print(e)

    def convert_depth_to_point(self, u, v, depth):
        ##############################################################
        # From here
        fx = 700.0  # Focal length in x direction
        fy = 700.0  # Focal length in y direction
        cx = 640.0  # Principal point x
        cy = 360.0  # Principal point y
        # to here, needs to be configured with real value
        ##############################################################

        x = float((u - cx) * depth / fx)
        y = float((v - cy) * depth / fy)
        z = float(depth)
        # print(x, y, z)
        return [x, y, z]


def main(args=None):
    rclpy.init(args=args)
    depth_to_pc = DepthSubscriber()
    rclpy.spin(depth_to_pc)
    depth_to_pc.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
