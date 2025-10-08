from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'object_detect'

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
    maintainer='mtrn',
    maintainer_email='z5376231@ad.unsw.edu.au',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'object_detect = object_detect.object_detect:main',
            'camera_tf = object_detect.camera_frame_pub:main',
            'dummy_free = object_detect.dummy_free:main',
            'dummy_workspace = object_detect.dummy_workspace:main'
        ],
    },
)
