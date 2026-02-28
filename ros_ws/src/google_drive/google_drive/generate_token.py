import os
from google.auth.transport.requests import Request
from google.oauth2.credentials import Credentials
from google_auth_oauthlib.flow import InstalledAppFlow

# Scope: upload files to Google Drive
SCOPES = ['https://www.googleapis.com/auth/drive.file']

def main():
    creds = None

    # Load existing token if it exists
    if os.path.exists('token.json'):
        creds = Credentials.from_authorized_user_file('token.json', SCOPES)

    # If no valid credentials, force login
    if not creds or not creds.valid:
        if creds and creds.expired and creds.refresh_token:
            creds.refresh(Request())
        else:
            flow = InstalledAppFlow.from_client_secrets_file(
                'credentials.json',
                SCOPES
            )
            creds = flow.run_local_server(port=0)

        # CREATE token.json here
        with open('token.json', 'w') as token:
            token.write(creds.to_json())

    print("Token generated successfully")
    print("token.json created in this folder")

if __name__ == '__main__':
    main()
