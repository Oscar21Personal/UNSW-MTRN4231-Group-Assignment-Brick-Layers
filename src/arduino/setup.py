from setuptools import find_packages, setup
import os
from glob import glob


package_name = 'serial_listener'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*')))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='tanmeet',
    maintainer_email='tanmeet.sachdeva@gmail.com',
    description='A ROS 2 package to listen to Arduino serial data',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        'serial_listener_node = serial_listener.serial_listener_node:main',
        ],
    },
)
