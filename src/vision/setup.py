from setuptools import setup, find_packages

setup(
    name='vision',
    version='0.0.0',
    packages=find_packages(include=['vision', 'vision.*']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/vision']),
        ('share/vision', ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    entry_points={
        'console_scripts': [
            'detector = vision.detector:main',
            'bottom_camera = vision.bottom_camera_node:main',
            'behavior_status_listener = vision.movement:main',
        ],
    },
)