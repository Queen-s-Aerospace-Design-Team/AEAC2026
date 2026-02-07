import os
from glob import glob
from setuptools import find_packages, setup

PKG_NAME = 'navigation_core'

setup(
    name=PKG_NAME,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + PKG_NAME]),
        ('share/' + PKG_NAME, ['package.xml']),
        (os.path.join('share', PKG_NAME, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='qadt',
    maintainer_email='anthonyjb5228@gmail.com',
    description='Core navigation functionality for UAV',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'cmd_vel_to_px4 = navigation_core.cmd_vel_to_px4:main',
            'cmd_vel_test = navigation_core.cmd_vel_test:main',
            'pointcloud_manipulate = navigation_core.pointcloud_manipulate:main',
        ],
    },

)
