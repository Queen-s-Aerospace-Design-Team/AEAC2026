import rclpy
from rclpy.node import Node
from std_msgs.msg import String
 
import sys
import os
from upload_photo import upload_image
 
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
 
 
#configure Qos for publishing and subscribing
class drive_upload_service(Node):
    def __init__(self):
        super().__init__('drive_upload_service')
 
    qos_profile = QoSProfile(
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.TRANSIENT_LOCAL,
        history=HistoryPolicy.KEEP_LAST,
        depth=1,
    )
   
 
# I don't believe we need any publishers for this node
#create publishers
#self.offboard_control_mode_publisher = self.create_publisher(
            #OffboardControlMode, '/fmu/in/offboard_control_mode', qos_profile)
 
 
 
 
#create subscribers
#I am not sure which part of this is correct, but I think they both work, they are just two different versions.
self.drive_upload_subscriber = self.create_subscription(
    String,                 # Message type --> std_msgs/msg/String
    '/drive_upload',        # Topic name
    self.drive_upload_callback, # Callback function
    10                      # Queue size (QoS depth)
)
 
#self.drive_upload_subscriber = self.create_subscription(
    #String, '/drive.upload', self.drive_upload_callback, qos_profile)
 
 
 
#initialize variables
 
 
#define functions
def drive_upload_callback(self, msg):
    #upload media file to google drive
    self.get_logger().info(f"I heard: {msg.data}")
 
 
    # this is the part I'm really not sure about, and how it works
    #this is supposed to be the part that triggers the upload_image() funciton in upload_photo.py
   
    # I thought I was supposed to use the commands:
    #scp media_file, ges "secure copy"
    #ssh ges python3 uploadMedia.py
 
    file_path = msg.data
 
    self.get_logger().info(f"Received: {file_path}")
 
    try:
        upload_image()
        self.get_logger().info("Upload successful")
 
    except Exception as e:
        self.get_logger().error(f"Upload failed: {e}")
 
 
 
 
#main loop
def main(args=None):
    rclpy.init(args=args)
 
    node = MySubscriber()
    rclpy.spin(node)
 
    node.destroy_node()
    rclpy.shutdown()
 
#not sure which one is right --> both are very similar
#def main():
   #rospy.init_node('drive_upload_node')
 
   #rospy.Subscriber("/image_path", String, image_callback)
 
   #rospy.loginfo("Drive upload node started")
   #rospy.spin()
 
 
if __name__ == '__main__':
    main()
   