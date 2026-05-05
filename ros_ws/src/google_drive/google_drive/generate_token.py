from pathlib import Path

from google.auth.transport.requests import Request
from google.oauth2.credentials import Credentials
from google_auth_oauthlib.flow import InstalledAppFlow

# Scope: upload files created by this app to Google Drive
SCOPES = ["https://www.googleapis.com/auth/drive.file"]

# For devcontainer / Jetson / headless use.
# This must match the redirect URI used in the generated auth URL.
REDIRECT_URI = "http://localhost:8080/"

def find_repo_root() -> Path:
    """
    Find the repo root by walking upward from the current working directory
    and from this file's path.

    Expected layout:
        AEAC2026/
        ├── .credentials/
        │   ├── credentials.json
        │   └── token.json
        └── ros_ws/
            └── src/
                └── google_drive/
                    └── google_drive/
                        └── generate_token.py
    """
    search_roots = [Path.cwd(), Path(__file__).resolve()]

    for root in search_roots:
        for parent in [root, *root.parents]:
            if (parent / ".credentials").is_dir():
                return parent

    # Fallback: use current working directory if .credentials does not exist yet.
    return Path.cwd()


def get_credentials_paths() -> tuple[Path, Path]:
    repo_root = find_repo_root()
    credentials_dir = repo_root / ".credentials"
    credentials_path = credentials_dir / "credentials.json"
    token_path = credentials_dir / "token.json"

    return credentials_path, token_path


def generate_new_credentials(credentials_path: Path) -> Credentials:
    flow = InstalledAppFlow.from_client_secrets_file(
        str(credentials_path),
        SCOPES,
    )

    flow.redirect_uri = REDIRECT_URI

    auth_url, _ = flow.authorization_url(
        access_type="offline",
        prompt="consent",
        include_granted_scopes="true",
    )

    print("\nOpen this URL in your Mac browser:\n")
    print(auth_url)

    print(
        "\nAfter approving access, your browser will redirect to a localhost URL."
    )
    print(
        "The page may fail to load. That is expected in a devcontainer/headless setup."
    )
    print(
        "\nCopy the FULL redirected URL from the browser address bar and paste it below."
    )
    print("It should look like:")
    print("http://localhost:8080/?code=...&scope=...\n")

    redirected_url = input("Paste full redirected URL here: ").strip()

    if not redirected_url:
        raise RuntimeError("No redirected URL was provided.")

    flow.fetch_token(authorization_response=redirected_url)

    return flow.credentials


def main() -> None:
    creds = None

    credentials_path, token_path = get_credentials_paths()
    credentials_dir = credentials_path.parent

    if not credentials_path.exists():
        raise FileNotFoundError(
            f"Missing Google OAuth client file: {credentials_path}\n"
            "Expected credentials.json in the .credentials directory."
        )

    if token_path.exists():
        creds = Credentials.from_authorized_user_file(str(token_path), SCOPES)

    if creds and creds.valid:
        print("Existing token is valid")
        print(f"Token path: {token_path}")
        return

    if creds and creds.expired and creds.refresh_token:
        print("Refreshing expired token...")
        creds.refresh(Request())
    else:
        print("Generating new Google OAuth token...")
        creds = generate_new_credentials(credentials_path)

    credentials_dir.mkdir(parents=True, exist_ok=True)

    with token_path.open("w") as token:
        token.write(creds.to_json())

    print("\nToken generated successfully")
    print(f"Token path: {token_path}")


if __name__ == "__main__":
    main()