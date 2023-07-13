import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, PointField
from visualization_msgs.msg import Marker
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Point
from geometry_msgs.msg import Quaternion
from nav_msgs.msg import GridCells

import math

######################################################
# can simply import open3d by
#    pip install open3d
import open3d as o3d

######################################################


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


def quaternion_to_rotmat(o4x1: np.array) -> np.array:
    # quaternion to rotation matrix
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


def view_mx(orientation_4x1, pos_1x3):
    rot3x3 = quaternion_to_rotmat(orientation_4x1)
    view_matrix = np.identity(4)
    view_matrix[:3, :3] = rot3x3.T
    view_matrix[:3, 3] = -np.dot(rot3x3.T, pos_1x3)
    return view_matrix


def change_to_int(point: list, size: float) -> tuple:
    fx = math.floor(point[0] / size)
    fy = math.floor(point[1] / size)
    fz = math.floor(point[2] / size)
    return int(fx), int(fy), int(fz)


class PointCloudSubscriber(Node):
    def __init__(self):
        super().__init__("pointcloud_subscriber")

        print("================map_talker initiated!================")

        # pointcloud2 msg 읽어옴
        self.point_subscription = self.create_subscription(
            PointCloud2,
            "/zed2/zed_node/point_cloud/cloud_registered",
            self.pointcloud_callback,
            10,
        )
        self.point_subscription  # prevent unused variable warning

        # pose msg 읽어옴
        self.pose_subscription = self.create_subscription(
            PoseStamped, "/zed2/zed_node/pose", self.pose_callback, 10
        )
        self.pose_subscription  # prevent unused variable warning

        # self.camera_pose_publisher = self.create_publisher(
        #     PoseStamped, "/map_talker/camera_pose", 10
        # )

        self.grid_publisher = self.create_publisher(
            GridCells, "/map_talker/gridcells", 10
        )

        self.marker_publisher = self.create_publisher(Marker, "/map_talker/marker", 10)

        self.pos = None

        self.pcd = o3d.geometry.PointCloud()
        self.voxel_size = 0.2

    def pointcloud_callback(self, msg):
        if self.pos is None:
            return

        ######################################################
        # zed camera coordinate system points
        points = read_points(msg)
        points = points.reshape(len(points), 4, 1)

        ######################################################
        # making zed camera coordinate system points
        # to zed world coordinate system points
        # points = np.matmul(np.linalg.inv(view_mx(self.orientation, self.pos)), points)
        # points = points.reshape(len(points), 4)
        points = points[:, :3, :]
        rotmat = quaternion_to_rotmat(self.orientation)
        points = np.matmul(rotmat, points) + self.pos.reshape(3, 1)

        ######################################################
        ######################################################
        # NOTICE : I'VE NEVER VISUALIZED OR CHECKED NED CONVERSION(THE CODE BELOW THIS NOTICE) YET
        # NOTICE : I'VE NEVER VISUALIZED OR CHECKED NED CONVERSION(THE CODE BELOW THIS NOTICE) YET
        ######################################################
        ######################################################
        # making zed world coordinate system points
        # to NED world coordinate system points
        #
        # in this code, I was trying to do this.
        #
        #   N                 1  0  0     cx     tx
        #   E  =  quatmat  x  0 -1  0  x  cy  +  ty
        #   D                 0  0 -1     cz     tz
        #
        # cmat = np.array(
        #     [
        #         [1, 0, 0],
        #         [0, -1, 0],
        #         [0, 0, -1],
        #     ]
        # )
        # quatmat = quaternion_to_rotmat(self.orientation)
        # points = np.matmul(cmat, points)
        # points = np.matmul(quatmat, points)
        # points += self.pos.reshape(3, 1)
        ######################################################
        # downsampling pointcloud using open3d
        points = points.reshape(len(points), 3)

        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(points)
        self.pcd += pcd
        self.pcd = self.pcd.voxel_down_sample(voxel_size=self.voxel_size)

        self.marker_publish(msg.header.frame_id)
        self.grid_publish(msg.header.frame_id)

    def grid_publish(self, frame_id):
        if self.pos is None:
            return

        grid_cells_msg = GridCells()
        grid_cells_msg.header.frame_id = frame_id
        grid_cells_msg.cell_width = self.voxel_size
        grid_cells_msg.cell_height = self.voxel_size

        threshold_lower = self.pos[2] - self.voxel_size * 0.5
        threshold_upper = self.pos[2] + self.voxel_size * 0.5

        points = np.asarray(self.pcd.points)

        selected_points = points[
            (points[:, 2] > threshold_lower) & (points[:, 2] < threshold_upper)
        ]

        for point in selected_points:
            p = Point()
            p.x = point[0]
            p.y = point[1]
            p.z = point[2]
            grid_cells_msg.cells.append(p)
        print("grid cell num: ", len(selected_points))
        self.grid_publisher.publish(grid_cells_msg)

    # only for checking(visualization)
    def marker_publish(self, frame_id):
        marker = Marker()
        marker.header.frame_id = frame_id
        marker.type = Marker.CUBE_LIST
        marker.action = Marker.DELETEALL
        marker.scale.x = self.voxel_size
        marker.scale.y = self.voxel_size
        marker.scale.z = self.voxel_size
        marker.color.a = 1.0
        marker.color.r = 1.0
        marker.color.g = 1.0
        marker.color.b = 1.0
        self.marker_publisher.publish(marker)

        marker = Marker()
        marker.header.frame_id = frame_id
        marker.type = Marker.CUBE_LIST
        marker.action = Marker.ADD
        marker.scale.x = self.voxel_size
        marker.scale.y = self.voxel_size
        marker.scale.z = self.voxel_size
        marker.color.a = 1.0
        marker.color.r = 1.0
        marker.color.g = 1.0
        marker.color.b = 1.0

        points = np.asarray(self.pcd.points)
        for point in points:
            p = Point()
            p.x = point[0]
            p.y = point[1]
            p.z = point[2]
            marker.points.append(p)

        self.marker_publisher.publish(marker)

    def pose_callback(self, msg):
        camera_position = msg.pose.position
        camera_orientation = msg.pose.orientation

        self.pos = np.array([camera_position.x, camera_position.y, camera_position.z])
        self.orientation = np.array(
            [
                camera_orientation.x,
                camera_orientation.y,
                camera_orientation.z,
                camera_orientation.w,
            ]
        )

        # only for checking. not used
        # camera_pose_msg = PoseStamped()
        # camera_pose_msg.header = msg.header
        # camera_pose_msg.pose.position = camera_position
        # camera_pose_msg.pose.orientation = camera_orientation
        # self.camera_pose_publisher.publish(camera_pose_msg)


def main(args=None):
    rclpy.init(args=args)
    pointcloud_subscriber = PointCloudSubscriber()
    rclpy.spin(pointcloud_subscriber)
    pointcloud_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
