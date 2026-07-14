import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'imu'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
            glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'rviz'),
            glob('rviz/*.rviz')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='robosub',
    maintainer_email='robosub@robosub.com',
    description='ZED 2i IMU orientation visualization for RViz.',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'orientation_node = imu.orientation_node:main',
            'diagnostics_node = imu.diagnostics_node:main',
            'marker_node = imu.marker_node:main',
        ],
    },
)
