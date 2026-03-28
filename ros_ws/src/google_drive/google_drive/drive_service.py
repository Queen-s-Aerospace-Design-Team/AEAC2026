#converting the drive_uploader.py code into a service 

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from std_srvs.srv import SetBool #might not need this as we have the srv filefrom googleapiclient.http import MediaFileUpload
import os

from google.oauth2.credentials import Credentials
from googleapiclient.discovery import build
from googleapiclient.http import MediaFileUpload
from google_drive.srv import drive_uploader


 
#convert this into a service node 
class DriveUploadService(Node):
    def __init__(self):
        super().__init__('drive_upload_service')
 
        self.SCOPES = ['https://www.googleapis.com/auth/drive.file']
        self.FOLDER_ID = '1yy-xZOe-KUnGH505fWoLm3GsgbwL8CR_' #replace this with real folder id

        self.srv = self.create_service(
            drive_uploader,
            'upload_to_drive',
            self.upload_callback
        )
        self.get_logger().info("Drive uploader service is online, awaiting file paths...")

    #logic replaced from upload_photo.py here 
    #uploads the image to google drive
    def upload_image(self,path):
        creds = Credentials.from_authorized_user_file('token.json', self.SCOPES)
        service = build('drive', 'v3', credentials=creds)

        #get the basename of the file from the path
        #could just replace with file_name = path
        file_name = os.path.basename(path)

        file_metadata = {
            'name': file_name,
            'parents': [self.FOLDER_ID]   #tells what folder to paste it in
        }

        media = MediaFileUpload(
            path,
            mimetype='image/jpeg',
            resumable=True
        )

        file = service.files().create(
            body=file_metadata,
            media_body=media,
            fields='id'
        ).execute()

        return file.get('id')
    
    #can exclude the response.message lines if we don't want them, just here for debugging. 
    #would need to update the srv file to exclude 'message'
    def upload_callback(self, request, response):
        requested_path = request.file_path

        self.get_logger().info(f"Received upload request for: {requested_path}")

        #Check file exists
        if not os.path.exists(requested_path):
            response.success = False
            response.message = f"Error file {requested_path} not found"
            return response
        
        try: 
            file_id = self.upload_image(requested_path)
            response.success = True 
            response.message = f"Success, google drive ID: {file_id}"
            self.get_logger().info(f'Upload complete for {requested_path}')
        except Exception as e: 
            response.success = False
            response.message = f"Upload failed: {str(e)}"
            self.get_logger().error(response.message)

        return response

def main(args=None):
    rclpy.init(args=args)
    node = DriveUploadService()
    try: 
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()