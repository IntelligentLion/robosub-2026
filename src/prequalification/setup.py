from setuptools import find_packages, setup

package_name = 'prequalification'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', [
            'launch/prequalification.launch.py',
            'launch/prequal_dry_test.launch.py',
        ]),
        ('share/' + package_name + '/config', [
            'config/prequalification.yaml',
        ]),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='robosub',
    maintainer_email='robosub@robosub.com',
    description='Scripted RoboSub 2026 prequalification run',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'prequalification_node = prequalification.prequalification_node:main',
            'prequalification_dry_test = '
            'prequalification.prequal_dry_test_node:main',
        ],
    },
)
