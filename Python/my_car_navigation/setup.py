from setuptools import setup
import os
from glob import glob

package_name = 'my_car_navigation'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        # ROS 2 metadata
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),

        # 🚀 Install the launch folder
        (os.path.join('share', package_name, 'launch'),
            glob('launch/*.py')),

        # (optional) Install config folder if you have Nav2 params
        (os.path.join('share', package_name, 'config'),
            glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='rhys',
    maintainer_email='you@example.com',
    description='On-the-fly navigation node for car using Nav2',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # name: path.to.module:main
            'navigation_node = my_car_navigation.navigation_node:main',
        ],
    },
)
