import numpy as np
import sample.sample.seung_calc as seung_calc

pixel_x = 816
pixel_y = 640


qx = 0
qy = 0 
qz = 0
qw = 0

tx = 0
ty = 0
tz = 0

px = 0
py = 0 
pz = 0
 
yolo_x = 0
yolo_y = 0


class sehyun_Node(Node):
    def __init__(self):
        super().__init__('sehyun_node')
        
        subscribe_yolo = self.create_subscription(msg_type, 'yolo_msg', self.callback_yolo, 10)
        subscribe_pose = self.create_subscription(msg_type, '/zed2/zed_/pose', self.callback_pose, 10)
        subscribe_pointcloud = self.create_subscription(msg_type, '/zed2/zed_node/point', self.callback_pointcloud, 10)
        
        Q = seung_calc.Q2M(qx, qy, qz, qw)
        T = seung_calc.Transform(Q, px, py, pz, tx, ty, tz)
        
        sehyun_pub = self.create_publish(msg_type, 'seung2sehyn', 10)
        
    def publisher(self):
        msg = poistion()
        msg.stamp = self.get_clock().now().to_msg
        msg.x = ?
        msg.y = ?
        msg.z = ?
        self.publisher.publish(msg)
        self.get_logger().info('Published x = {0}'.format(msg.x))   
        self.get_logger().info('Published y = {0}'.format(msg.y))   
        self.get_logger().info('Published z = {0}'.format(msg.z))  
        self.publisher = self.create_publisher(
            points,
            's2s_topic',
            qos)

    
    def callback_yolo(self, yolo_msg):
        yolo_x = yolo_msg.x_pixel
        yolo_y = yolo_msg.y_pixel
    
    def callback_pose(self, pose_msg):
        qx = pose_msg.x
        qy = pose_msg.y
        qz = pose_msg.z
        qw = pose_msg.w
    
        tx = pose_msg.position.x
        ty = pose_msg.position.y
        tz = pose_msg.position.z

    
    def callback_pointcloud(self, pointCloud2_msg):
        array = []
        array = pointCloud2_msg.data
        
        #np_array = np.array
        #index = calc.refine_resol(yolo_x, yolo_y)
        index = ((yolo_y-1) * pixel_y + yolo_x - 1) * 16        
        
        px = array[index:index+3]
        py = array[index+4:index+7]
        pz = array[index+8:index+11]

    
        
         
          
def main(args=None):
    rclpy.init
    rclpy.spin(sehyun_Node)
    
    