import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'google_drive'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'test_data'), ['google_drive/test_image.jpg']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='qadt',
    maintainer_email='anthonyjb5228@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'drive_uploader = google_drive.drive_uploader:main',
            'drive_upload_test_client = google_drive.drive_upload_test_client:main',
        ],
    },
)
