import numpy as np
import rclpy
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


def change_to_int(point, size):
    fx = math.floor(point[0] / size)
    fy = math.floor(point[1] / size)
    fz = math.floor(point[2] / size)
    return int(fx), int(fy), int(fz)


class PointCloudManager:
    def __init__(self):
        self.len = 0
        self.voxel_size = 0.2
        self.voxel_dict = {}
        self.voxel_arr_fordraw = []

    def push_points(self, points):
        self.voxel_arr_fordraw = []

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

            self.voxel_arr_fordraw.append(
                Point(
                    x=float(x) * self.voxel_size + self.voxel_size * 0.5,
                    y=float(y) * self.voxel_size + self.voxel_size * 0.5,
                    z=float(z) * self.voxel_size + self.voxel_size * 0.5,
                )
            )
            continue
            if z not in self.voxel_dict:
                self.voxel_dict[z] = {}
            xy_mixed = x << 16 | y
            if xy_mixed in self.voxel_dict[z]:
                self.voxel_dict[z][xy_mixed] += 1
            else:
                self.voxel_dict[z][xy_mixed] = 1
                self.voxel_arr_fordraw.append(
                    Point(
                        x=float(x) * self.voxel_size + self.voxel_size * 0.5,
                        y=float(y) * self.voxel_size + self.voxel_size * 0.5,
                        z=float(z) * self.voxel_size + self.voxel_size * 0.5,
                    )
                )
                self.len += 1
        print("array  len: ", len(self.voxel_arr_fordraw))
        return


class PointCloudSubscriber(Node):
    def __init__(self):
        super().__init__("pointcloud_subscriber")
        self.point_subscription = self.create_subscription(
            PointCloud2,
            "/zed2/zed_node/mapping/fused_cloud",
            self.pointcloud_callback,
            10,
        )
        self.point_subscription  # prevent unused variable warning
        self.pose_subscription = self.create_subscription(
            PoseStamped, "/zed2/zed_node/pose", self.pose_callback, 10
        )
        self.pose_subscription  # prevent unused variable warning

        self.camera_pose_publisher = self.create_publisher(
            PoseStamped, "/map_talker/camera_pose", 10
        )
        self.marker_publisher = self.create_publisher(Marker, "/map_talker/marker", 10)
        self.pointcloud_publisher = self.create_publisher(
            PointCloud2, "/map_talker/pc2", 10
        )

        self.pcm = PointCloudManager()

        self.counter = 0

    def pointcloud_callback(self, msg):
        pcmsg = PointCloud2()
        pcmsg.header.frame_id = msg.header.frame_id
        pcmsg.height = msg.height
        pcmsg.width = msg.width
        pcmsg.fields = msg.fields
        pcmsg.is_bigendian = msg.is_bigendian
        pcmsg.point_step = msg.point_step
        pcmsg.row_step = msg.row_step
        pcmsg.data = msg.data

        self.pointcloud_publisher.publish(pcmsg)

        points = read_points(msg)
        print("points len: ", len(points))

        # file_path = "output%d.txt" % self.counter
        # self.counter += 1
        # with open(file_path, "w") as file:
        #     for point in points:
        #         x, y, z = point[0], point[1], point[2]
        #         if (
        #             np.isinf(x)
        #             or np.isinf(y)
        #             or np.isinf(z)
        #             or np.isnan(x)
        #             or np.isnan(y)
        #             or np.isnan(z)
        #         ):
        #             continue

        #         line = f"{x} {y} {z}\n"
        #         file.write(line)

        # points = points.reshape(len(points), 4, 1)
        points = np.matmul(np.linalg.inv(view_mx(self.orientation, self.pos)), points)

        self.pcm.push_points(points)
        # print("pcm.len: ", self.pcm.len)

        self.marker_publish()

    def marker_publish(self):
        marker = Marker()
        marker.header.frame_id = "base_link"
        marker.type = Marker.CUBE_LIST
        marker.action = Marker.DELETEALL
        marker.scale.x = self.pcm.voxel_size
        marker.scale.y = self.pcm.voxel_size
        marker.scale.z = self.pcm.voxel_size
        marker.color.a = 1.0
        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0
        self.marker_publisher.publish(marker)

        marker = Marker()
        marker.header.frame_id = "base_link"
        marker.type = Marker.CUBE_LIST
        marker.action = Marker.ADD
        marker.scale.x = self.pcm.voxel_size
        marker.scale.y = self.pcm.voxel_size
        marker.scale.z = self.pcm.voxel_size
        marker.color.a = 1.0
        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0
        marker.points = self.pcm.voxel_arr_fordraw
        self.marker_publisher.publish(marker)

    def pose_callback(self, msg):
        camera_position = msg.pose.position
        camera_orientation = msg.pose.orientation

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


i = 0


def view_mx(orientation_4x1, pos_1x3):
    rot3x3 = Q2M(orientation_4x1)
    view_matrix = np.identity(4)
    view_matrix[:3, :3] = rot3x3.T
    view_matrix[:3, 3] = -np.dot(rot3x3.T, pos_1x3)
    # global i
    # if not i:
    #     print("view_matrix: \n", view_matrix)
    # i += 1

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
