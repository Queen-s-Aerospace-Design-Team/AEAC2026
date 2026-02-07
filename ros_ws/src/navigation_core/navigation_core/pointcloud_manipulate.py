import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2

class pointcloud_manipulate(Node):
    def __init__(self):
        super().__init__('pointcloud_manipulate')

        self.subscription = self.create_subscription(
            PointCloud2,
            '/depth_camera/points',
            self.pointcloud_callback,
            10
        )

        self.get_logger().info('PointCloud Processor node started')

    def pointcloud_callback(self, msg):
        self.get_logger().info(
            f'Received point cloud with {msg.width * msg.height} points'
        )

def main(args=None):
    rclpy.init(args=args)
    node = pointcloud_manipulate()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
