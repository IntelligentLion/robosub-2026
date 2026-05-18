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
            'detector = vision.detector:main', #custom model
            'behavior_status_listener = vision.movement:main',
            #'detector = vision.obj_det:main', #preoptimized zed sample model
        ],
    },
)