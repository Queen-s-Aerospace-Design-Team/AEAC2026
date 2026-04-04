import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from .upload_photo import upload_image # .upload_photo 
 
class drive_uploader(Node):
    def __init__(self):
        super().__init__('drive_upload_service')
 
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        
        # I don't believe we need any publishers for this node
        # create publishers
        # self.offboard_control_mode_publisher = self.create_publisher(
                    #OffboardControlMode, '/fmu/in/offboard_control_mode', qos_profile)
        
        # create subscribers
        # Create service
        self.srv = self.create_service(
            UploadPhoto,
            'drive_upload',   # service name
            self.drive_upload_callback
        )

        self.get_logger().info("Drive upload service ready.")



 
    # Upload media file to google drive
    def drive_upload_callback(self, request, response): 
    
        # this is the part I'm really not sure about, and how it works
        # this is supposed to be the part that triggers the upload_image() funciton in upload_photo.py
    
        file_path = request.file_path
    
        self.get_logger().info(f"Received: {file_path}")
    
        try:
            upload_image(file_path)
            response.sucess = True
            response.message ="Upload successful"
            self.get_logger().info("Upload Successful")
        except Exception as e:
            response.sucess = False
            response.message = str(e)
            self.get_logger().error(f"Upload failed: {e}")
        
         return response
 

def main(args=None):
    rclpy.init(args=args)
    node = drive_uploader()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

