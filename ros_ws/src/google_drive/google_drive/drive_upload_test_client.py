import rclpy
from google_drive_interfaces.srv import DriveUploader
from rclpy.node import Node


class DriveUploadTestClient(Node):
    def __init__(self):
        super().__init__('drive_upload_test_client')
        self.declare_parameter('file_path', '')
        self.client = self.create_client(DriveUploader, 'upload_to_drive')

    def call_service(self):
        file_path = self.get_parameter('file_path').get_parameter_value().string_value

        if not file_path:
            self.get_logger().error('Missing file_path parameter.')
            return False

        self.get_logger().info('Waiting for upload_to_drive service...')
        if not self.client.wait_for_service(timeout_sec=10.0):
            self.get_logger().error('upload_to_drive service is not available.')
            return False

        request = DriveUploader.Request()
        request.file_path = file_path

        future = self.client.call_async(request)
        rclpy.spin_until_future_complete(self, future)

        if future.result() is None:
            self.get_logger().error(f'Service call failed: {future.exception()}')
            return False

        response = future.result()
        if response.success:
            self.get_logger().info(response.message)
        else:
            self.get_logger().error(response.message)

        return response.success


def main(args=None):
    rclpy.init(args=args)
    node = DriveUploadTestClient()

    try:
        node.call_service()
    finally:
        node.destroy_node()
        rclpy.shutdown()
