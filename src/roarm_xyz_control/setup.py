from setuptools import find_packages, setup
from glob import glob

package_name = 'roarm_xyz_control'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.py')),
    ],
    install_requires=['setuptools', 'pyserial'],
    zip_safe=True,
    maintainer='htsun',
    maintainer_email='your_email@example.com',
    description='ROS2 keyboard xyz+t control for RoArm-M2-S using JSON serial commands',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'keyboard_target_node = roarm_xyz_control.keyboard_target_node:main',
            'roarm_json_driver_node = roarm_xyz_control.roarm_json_driver_node:main',
            'lego_detector_node = roarm_xyz_control.lego_detector_node:main',
            'uv_to_arm_node = roarm_xyz_control.uv_to_arm_node:main',
        ],
    },
)