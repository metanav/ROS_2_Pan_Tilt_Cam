from setuptools import find_packages, setup
from glob import glob
import os

package_name = 'hand_tracker'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/resource',  glob(os.path.join('resource', '*.eim'))),
        ('share/' + package_name + '/launch',  ['launch/object_detection_node.launch.py', 'launch/hand_tracker_node.launch.py']),

    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ubuntu',
    maintainer_email='ubuntu@todo.todo',
    description='TODO: Package description',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'object_detection_node = hand_tracker.object_detection_node:main',
            'image_bbox_node = hand_tracker.image_bbox_node:main',
            'hand_tracker_node = hand_tracker.hand_tracker_node:main'
        ],
    },
)
