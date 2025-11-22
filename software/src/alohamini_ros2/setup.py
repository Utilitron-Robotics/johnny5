from setuptools import setup
import os
from glob import glob

package_name = 'alohamini_ros2'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='AlohaMini User',
    maintainer_email='user@todo.todo',
    description='ROS 2 Wrapper for AlohaMini / LeKiwi Robots',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'aloha_node = alohamini_ros2.aloha_node:main',
        ],
    },
)