from setuptools import find_packages, setup


package_name = 'mavlink_thruster_control'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='robosub',
    maintainer_email='robosub@robosub.com',
    description='Control Pixhawk thrusters via ROS 2 and pymavlink',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'thruster_node = mavlink_thruster_control.thruster_node:main',
            'safety_monitor_node = mavlink_thruster_control.safety_monitor_node:main',
        ],
    },
)
