from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'face_tracker'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        # 安装 msg
        (os.path.join('share', package_name, 'msg'), glob('msg/*.msg')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='hyperdrant',
    maintainer_email='sas573920@gmail.com',
    description='Simple face tracker publishing normalized offset (ux, uy) using OpenCV.',
    license='Apache License 2.0',
    entry_points={
        'console_scripts': [
            'tracker_node = face_tracker.tracker_node:main',
            'controller_node = face_tracker.controller_node:main',
            'fake_driver_node = face_tracker.fake_driver_node:main',
        ],
    },
)

