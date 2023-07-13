import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
import std_msgs.msg as std_msgs

import numpy as np
import open3d as o3d

import socket


def read_points(msg):
    # Get field information
    fields = msg.fields
    field_names = [field.name for field in fields]
    # field_offsets = [field.offset for field in fields]

    # Find the x, y, z fields
    x_idx = field_names.index("x")
    y_idx = field_names.index("y")
    z_idx = field_names.index("z")

    # Get data size and point step size
    data = msg.data
    # data_size = len(data)
    point_step = msg.point_step

    # Extract x, y, z coordinates from the data using np.frombuffer()
    data_buffer = np.frombuffer(data, dtype=np.float32)

    # Reshape the data buffer to match the structure of the point cloud
    point_data = data_buffer.reshape(-1, point_step // 4)

    # Perform point validity checks and create points array
    mask = (
        np.isfinite(point_data[:, x_idx])
        & np.isfinite(point_data[:, y_idx])
        & np.isfinite(point_data[:, z_idx])
        & (point_data[:, x_idx] < 14.0)
        & (point_data[:, x_idx] > -14.0)
        & (point_data[:, y_idx] < 14.0)
        & (point_data[:, y_idx] > -14.0)
    )
    valid_points = point_data[mask]

    return np.hstack(
        (valid_points[:, :3], np.ones((valid_points.shape[0], 1), dtype=np.float32))
    )


class PointCloudModifierNode(Node):
    def __init__(self):
        super().__init__("point_cloud_modifier_node")
        print("================pc_talker initiated!================")

        self.subscription = self.create_subscription(
            # PointCloud2, "/depth_camera/points", self.point_cloud_callback, 10
            PointCloud2,
            "/zed2/zed_node/point_cloud/cloud_registered",
            self.point_cloud_callback,
            10,
        )
        self.publisher = self.create_publisher(
            PointCloud2, "output_point_cloud_topic", 10
        )

    def point_cloud_callback(self, msg):
        print("msg recieved!")

        new_msg = PointCloud2()
        new_msg.header.frame_id = "base_link"
        new_msg.height = msg.height
        new_msg.width = msg.width
        new_msg.fields = msg.fields
        new_msg.is_bigendian = msg.is_bigendian
        new_msg.point_step = msg.point_step
        new_msg.row_step = msg.row_step
        new_msg.is_dense = msg.is_dense
        new_msg.data = msg.data

        self.publisher.publish(new_msg)

        points = read_points(msg)
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(points[:, :3])
        o3d.io.write_point_cloud("pcd_from_msg.pcd", pcd, True)


def main(args=None):
    rclpy.init(args=args)
    node = PointCloudModifierNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
