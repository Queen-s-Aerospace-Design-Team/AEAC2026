import os
from google.oauth2.credentials import Credentials
from googleapiclient.discovery import build
from googleapiclient.http import MediaFileUpload

SCOPES = ['https://www.googleapis.com/auth/drive.file']

# 🔴 REPLACE THIS WITH YOUR REAL FOLDER ID
FOLDER_ID = '1yy-xZOe-KUnGH505fWoLm3GsgbwL8CR_'

def upload_image():
    creds = Credentials.from_authorized_user_file('token.json', SCOPES)
    service = build('drive', 'v3', credentials=creds)

    file_name = 'test_image.jpg'  # image must exist locally

    file_metadata = {
        'name': file_name,
        'parents': [FOLDER_ID]   #tells what folder to paste it in
    }

    media = MediaFileUpload(
        file_name,
        mimetype='image/jpeg',
        resumable=True
    )

    file = service.files().create(
        body=file_metadata,
        media_body=media,
        fields='id'
    ).execute()

    print(f"Uploaded to AEAC 2026 Targets ✅")
    print(f"File ID: {file.get('id')}")

if __name__ == '__main__':
    upload_image()



