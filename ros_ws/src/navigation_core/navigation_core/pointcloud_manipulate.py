import math
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, LaserScan
from sensor_msgs_py import point_cloud2


class pointcloud_manipulate(Node):
    def __init__(self):
        super().__init__('pointcloud_manipulate')

        #slice variables

        #height of the slice
        #heigh in meters
        self.min_height = 0.1
        self.max_height = 1.0
        
        #LaserScan config 
        #range is in meters
        self.range_min = 0.1
        self.range_max = 20.0
        self.angle_min = -math.pi # -180 deg
        self.angle_max = math.pi # 180 deg
        self.angle_change = math.pi / 360 #0.5 deg angle change
        self.num_bins = int(round((self.angle_max - self.angle_min) / self.angle_change)) #amount of indicies needed to store that data at each angle change


        #publishers
        self.scan_pub = self.create_publisher(
            LaserScan,
            '\scan'
            10
        )

        #subscribers
        self.subscription = self.create_subscription(
            PointCloud2, #message type
            '/depth_camera/points', #topic
            self.pointcloud_callback, #what to do when it receives the message from the subscribed node
            10 #Qualitly of service (QOS), ignore this
        )

        self.get_logger().info('PointCloud Processor node started')

    def _cloud_callback(self, cloud_msg: PointCloud2) -> None:
            pointers_iteration = pc2.read_points(
                cloud_msg, 
                field_names = ["x", "y", "z"],
                skip_nans = True
            )

            arr_coords = np.array(list(pointers_iteration), dtype=np.float32)
            #output: [[0,1,2], [3,4,5], ...]

            #edge cases incase there are no points  
            if len(arr_coords) == 0: 
                self.get_logger().warn("No points found in the point cloud") 
            self.get_logger().debug(f"Received point cloud with {len(arr_coords)} points")

            self.arr_coords = arr_coords


        

def main(args=None):
    rclpy.init(args=args)
    node = pointcloud_manipulate()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
