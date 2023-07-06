import rclpy
import numpy as np
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, PointField
from visualization_msgs.msg import Marker
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Point

import struct
import math


def read_points(msg):
    # Get field information
    fields = msg.fields
    field_names = [field.name for field in fields]
    field_offsets = [field.offset for field in fields]
    field_sizes = [field.count for field in fields]

    # Find the x, y, z fields
    x_idx = field_names.index("x")
    y_idx = field_names.index("y")
    z_idx = field_names.index("z")

    # Get data size
    data = msg.data
    data_size = len(data)

    # Calculate point step size
    point_step = msg.point_step

    points = []

    # Extract x, y, z coordinates from the data
    for i in range(0, data_size, point_step):
        x = struct.unpack_from("f", data, i + field_offsets[x_idx])[0]
        y = struct.unpack_from("f", data, i + field_offsets[y_idx])[0]
        z = struct.unpack_from("f", data, i + field_offsets[z_idx])[0]
        points.append(np.array([x, y, z, 1]))

    return np.array(points)


def change_to_int(point, size=1.0):
    fx = math.floor(point[0] / size)
    fy = math.floor(point[1] / size)
    fz = math.floor(point[2] / size)
    return int(fx), int(fy), int(fz)


class PointCloudManager:
    def __init__(self):
        self.len = 0
        self.voxel_size = 0.01
        self.voxel_dict = {}
        self.voxel_arr_fordraw = []

    def push_points(self, points):
        for point in points:
            x, y, z = point[0], point[1], point[2]
            if (
                np.isinf(x)
                or np.isinf(y)
                or np.isinf(z)
                or np.isnan(x)
                or np.isnan(y)
                or np.isnan(z)
            ):
                continue

            x, y, z = change_to_int(point, self.voxel_size)

            if z not in self.voxel_dict:
                self.voxel_dict[z] = {}
            xy_mixed = x << 16 | y
            if xy_mixed in self.voxel_dict[z]:
                self.voxel_dict[z][xy_mixed] += 1
            else:
                self.voxel_dict[z][xy_mixed] = 1
                self.voxel_arr_fordraw.extend(struct.pack("fff", x, y, z))
                self.len += 1


class PointCloudSubscriber(Node):
    def __init__(self):
        super().__init__("pointcloud_subscriber")
        self.point_subscription = self.create_subscription(
            PointCloud2,
            "/zed2/zed_node/point_cloud/cloud_registered",
            self.pointcloud_callback,
            10,
        )
        self.point_subscription  # prevent unused variable warning
        self.pose_subscription = self.create_subscription(
            PoseStamped, "/zed2/zed_node/pose", self.pose_callback, 10
        )
        self.pose_subscription  # prevent unused variable warning

        self.pc2_publisher = self.create_publisher(
            PointCloud2, "/map_talker/point_cloud", 10
        )
        self.pc2_publisher_multed = self.create_publisher(
            PointCloud2, "/map_talker/point_cloud_multed", 10
        )
        self.marker_publisher = self.create_publisher(
            Marker, "/map_talker/marker", 10
        )
        self.camera_pose_publisher = self.create_publisher(
            PoseStamped, "/map_talker/camera_pose", 10
        )

        self.pcm = PointCloudManager()

    def pointcloud_callback(self, msg):
        points = read_points(msg)
        points = points.reshape(len(points), 4, 1)
        print(points.shape)
        points = np.matmul(np.linalg.inv(ViewMX(self.orientation, self.pos)), points)

        self.pcm.push_points(points)
        print("self.pcm.len: ", self.pcm.len)
        pc2msg = PointCloud2()
        pc2msg.header.frame_id = "base_link"
        pc2msg.height = 1
        # pc2msg.width = len(points)
        pc2msg.width = self.pcm.len
        pc2msg.fields = [
            PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
        ]
        pc2msg.is_bigendian = False
        pc2msg.point_step = 12
        pc2msg.row_step = pc2msg.point_step * pc2msg.width
        pc2msg.is_dense = True

        # data_buffer = []

        # for point in points:
        #     x, y, z = point[0], point[1], point[2]
        #     data_buffer.extend(struct.pack("fff", x, y, z))

        # pc2msg.data = bytes(data_buffer)
        pc2msg.data = bytes(self.pcm.voxel_arr_fordraw)

        self.pc2_publisher_multed.publish(pc2msg)

    def pose_callback(self, msg):
        # Extract camera position and orientation from the PoseStamped message
        camera_position = msg.pose.position
        camera_orientation = msg.pose.orientation

        # Create a new PoseStamped message with the camera pose
        camera_pose_msg = PoseStamped()
        camera_pose_msg.header = msg.header
        camera_pose_msg.pose.position = camera_position
        camera_pose_msg.pose.orientation = camera_orientation

        self.pos = np.array([camera_position.x, camera_position.y, camera_position.z])
        self.orientation = np.array(
            [
                camera_orientation.x,
                camera_orientation.y,
                camera_orientation.z,
                camera_orientation.w,
            ]
        )

        # Publish the camera pose message
        self.camera_pose_publisher.publish(camera_pose_msg)

    def publish_pc2(self, msg):
        pc2msg = PointCloud2()
        pc2msg.header.frame_id = "base_link"
        pc2msg.height = msg.height
        pc2msg.width = msg.width
        pc2msg.fields = msg.fields
        pc2msg.is_bigendian = msg.is_bigendian
        pc2msg.point_step = msg.point_step
        pc2msg.row_step = msg.row_step
        pc2msg.is_dense = msg.is_dense
        pc2msg.data = msg.data

        self.pc2_publisher.publish(pc2msg)


def ViewMX(orientation_4x1, pos_1x3):
    rot3x3 = Q2M(orientation_4x1)
    view_matrix = np.identity(4)
    view_matrix[:3, :3] = rot3x3.T
    view_matrix[:3, 3] = -np.dot(rot3x3.T, pos_1x3)
    return view_matrix


def Q2M(o4x1):
    o4x1 /= np.linalg.norm(o4x1)
    x, y, z, w = o4x1

    rotation_matrix = np.array(
        [
            [1 - 2 * (y**2 + z**2), 2 * (x * y - w * z), 2 * (x * z + w * y)],
            [2 * (x * y + w * z), 1 - 2 * (x**2 + z**2), 2 * (y * z - w * x)],
            [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x**2 + y**2)],
        ]
    )

    return rotation_matrix


def main(args=None):
    rclpy.init(args=args)
    pointcloud_subscriber = PointCloudSubscriber()
    rclpy.spin(pointcloud_subscriber)
    pointcloud_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
