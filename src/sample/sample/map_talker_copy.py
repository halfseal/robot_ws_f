import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
from sensor_msgs.msg import PointCloud2
from sensor_msgs.msg import PointField
import numpy as np


# def nothing():
#         # PointCloud2 메시지 생성
#         msg = PointCloud2()
#         msg.header.frame_id = "base_link"
#         msg.height = 1
#         msg.width = len(points)
#         msg.fields.append(
#             pc2.PointField(name="x", offset=0, datatype=pc2.PointField.FLOAT32, count=1)
#         )
#         msg.fields.append(
#             pc2.PointField(name="y", offset=4, datatype=pc2.PointField.FLOAT32, count=1)
#         )
#         msg.fields.append(
#             pc2.PointField(name="z", offset=8, datatype=pc2.PointField.FLOAT32, count=1)
#         )
#         msg.is_bigendian = False
#         msg.point_step = 12
#         msg.row_step = msg.point_step * len(points)
#         msg.is_dense = True

#         # 포인트 데이터 채우기
#         msg.data = np.array(points, dtype=np.float32).tobytes()


class PointCloudPublisher(Node):
    def __init__(self):
        super().__init__("point_cloud_publisher")
        self.publisher = self.create_publisher(Marker, "voxel_marker", 10)
        self.timer_ = self.create_timer(1.0, self.timer_callback)

    def timer_callback(self, VOXEL_SIZE=1.0):
        # 텍스트 파일 읽기
        points = self.read_text_file("pointcloud2-11.txt")

        marker = Marker()
        marker.header.frame_id = "base_link"
        marker.type = Marker.CUBE_LIST  # 수정된 부분
        marker.action = Marker.ADD
        marker.scale.x = VOXEL_SIZE
        marker.scale.y = VOXEL_SIZE
        marker.scale.z = VOXEL_SIZE
        marker.color.a = 1.0
        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0

        points = [
            Point(x=0.0, y=0.0, z=0.0),
            Point(x=0.0, y=1.0, z=0.0),
            Point(x=0.0, y=0.0, z=1.0),
            Point(x=1.0, y=1.0, z=1.0),
            Point(x=1.0, y=0.0, z=1.0),
            Point(x=1.0, y=1.0, z=0.0),
            Point(x=1.0, y=0.0, z=0.0),
        ]

        # 정육각형의 꼭지점들을 marker의 points 필드에 추가합니다.
        marker.points = points

        self.publisher.publish(marker)

    def read_text_file(self, file_path):
        points = []
        with open(file_path, "r") as file:
            for line in file:
                point = line.strip().split(" ")
                x = float(point[0])
                y = float(point[1])
                z = float(point[2])
                points.append([x, y, z])
        return points


def main(args=None):
    rclpy.init(args=args)
    point_cloud_publisher = PointCloudPublisher()
    rclpy.spin(point_cloud_publisher)
    point_cloud_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
