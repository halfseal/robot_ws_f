# Populate the world with a json map file
#
import sys
import json
import rclpy
import random
from ament_index_python.packages import get_package_share_directory
from gazebo_msgs.srv import SpawnEntity, DeleteEntity


def make_obstacle(node, id, x0, y0, h, r):
    CYLINDER_MODEL = """
       <sdf version="1.6"> 				                      \
         <world name="default">                         \
           <model name="obstacle"> 		                  \
             <static>true</static> 		                	\
             <link name="all">                        	\
               <collision name="one">	              		\
                 <pose>0 0 {o} 0 0 0</pose>          		\
                 <geometry>				                      \
                   <cylinder>                         	\
                     <radius>{r}</radius>             	\
                     <length>{h}</length>              	\
                   </cylinder>   		                   	\
                  </geometry>			                    	\
               </collision>				                      \
               <visual name="two">		                	\
                 <pose>0 0 {o} 0 0 0</pose>          		\
                 <geometry>				                      \
                   <cylinder>                           \
                     <radius>{r}</radius>               \
                     <length>{h}</length>               \
                   </cylinder>                          \
                 </geometry>				                    \
               </visual>			                        	\
             </link>                                    \
           </model>				                            	\
         </world>                                       \
       </sdf>"""

    client = node.create_client(SpawnEntity, "/spawn_entity")
    node.get_logger().info("Connecting to /spawn_entity service...")
    client.wait_for_service()
    node.get_logger().info("...connected")
    request = SpawnEntity.Request()
    request.name = id
    request.initial_pose.position.x = float(x0)
    request.initial_pose.position.y = float(y0)
    request.initial_pose.position.z = float(0)
    dict = {"h": h, "r": r, "o": h / 2}
    request.xml = CYLINDER_MODEL.format(**dict)
    node.get_logger().info(f"Making request...")
    future = client.call_async(request)
    while not future.done():
        rclpy.spin_once(node)
    node.get_logger().info("...done")
    if not future.result().success:
        node.get_logger().info(f"Failure {future.result()}")


def main(args=None):
    rclpy.init(args=args)
    node = rclpy.create_node("build_map")

    for i in range(30):
        x = random.uniform(-5.0, 5.0)
        y = random.uniform(-5.0, 5.0)
        r = random.uniform(0.05, 0.2)
        node.get_logger().info(f"Populating map with {x} {y} {r}")
        make_obstacle(node, f"o{i}", x, y, 1.0, r)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
