import mimetypes
from pathlib import Path

import rclpy
from google.auth.transport.requests import Request
from google.oauth2.credentials import Credentials
from googleapiclient.discovery import build
from googleapiclient.http import MediaFileUpload
from google_drive_interfaces.srv import DriveUploader
from rclpy.node import Node

SCOPES = ['https://www.googleapis.com/auth/drive.file']
FOLDER_ID = '1yy-xZOe-KUnGH505fWoLm3GsgbwL8CR_'


def find_repo_root():
    search_roots = [Path.cwd(), Path(__file__).resolve()]

    for root in search_roots:
        for parent in [root, *root.parents]:
            if (parent / '.credentials').is_dir():
                return parent

    return Path.cwd()


class DriveUploadService(Node):
    def __init__(self):
        super().__init__('drive_upload_service')

        repo_root = find_repo_root()
        credentials_dir = repo_root / '.credentials'
        self.credentials_path = credentials_dir / 'credentials.json'
        self.token_path = credentials_dir / 'token.json'
        self.credentials = None
        self.credentials_error = None

        self.refresh_credentials_on_startup()

        self.srv = self.create_service(
            DriveUploader,
            'upload_to_drive',
            self.upload_callback
        )

        self.get_logger().info('Drive uploader service is online.')

    def refresh_credentials_on_startup(self):
        if not self.credentials_path.is_file():
            self.credentials_error = (
                f'Error: credentials file not found: {self.credentials_path}'
            )
            self.get_logger().error(self.credentials_error)
            return

        if not self.token_path.is_file():
            self.credentials_error = f'Error: token file not found: {self.token_path}'
            self.get_logger().error(self.credentials_error)
            return

        try:
            creds = Credentials.from_authorized_user_file(
                str(self.token_path),
                SCOPES
            )
        except Exception as exc:
            self.credentials_error = f'Error: failed to load token file: {exc}'
            self.get_logger().error(self.credentials_error)
            return

        if creds.valid:
            self.credentials = creds
            self.credentials_error = None
            self.get_logger().info('Google Drive token is valid.')
            return

        if creds.expired and creds.refresh_token:
            try:
                self.get_logger().info('Refreshing Google Drive token on startup...')
                creds.refresh(Request())
                self.token_path.write_text(creds.to_json())
                self.credentials = creds
                self.credentials_error = None
                self.get_logger().info('Google Drive token refreshed.')
                return
            except Exception as exc:
                self.credentials_error = (
                    f'Error: failed to refresh Google Drive token: {exc}'
                )
                self.get_logger().error(self.credentials_error)
                return

        self.credentials_error = (
            'Error: Google Drive token is invalid and cannot be refreshed. '
            'Run generate_token.py manually before deployment.'
        )
        self.get_logger().error(self.credentials_error)

    def upload_file(self, file_path):
        if self.credentials is None:
            raise RuntimeError(
                self.credentials_error or 'Google Drive token unavailable'
            )

        service = build('drive', 'v3', credentials=self.credentials)

        mimetype, _ = mimetypes.guess_type(file_path)
        media = MediaFileUpload(
            file_path,
            mimetype=mimetype or 'application/octet-stream',
            resumable=True
        )

        file_metadata = {
            'name': Path(file_path).name,
            'parents': [FOLDER_ID],
        }

        uploaded_file = service.files().create(
            body=file_metadata,
            media_body=media,
            fields='id'
        ).execute()

        return uploaded_file.get('id')

    def upload_callback(self, request, response):
        requested_path = Path(request.file_path).expanduser()

        self.get_logger().info(f'Received upload request for: {requested_path}')

        if not requested_path.is_file():
            response.success = False
            response.message = f'Error: file not found: {requested_path}'
            self.get_logger().error(response.message)
            return response

        if self.credentials is None:
            response.success = False
            response.message = (
                self.credentials_error or 'Error: Google Drive token unavailable'
            )
            self.get_logger().error(response.message)
            return response

        try:
            file_id = self.upload_file(str(requested_path))
            response.success = True
            response.message = f'Upload successful. Google Drive file ID: {file_id}'
            self.get_logger().info(response.message)
        except Exception as exc:
            response.success = False
            response.message = f'Upload failed: {exc}'
            self.get_logger().error(response.message)

        return response


def main(args=None):
    rclpy.init(args=args)
    node = DriveUploadService()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
