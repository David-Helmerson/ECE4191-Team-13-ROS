import os
from glob import glob
from setuptools import setup

package_name = 'perception'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']), 
        (os.path.join('share', package_name), glob('launch/*launch.[pxy][yma]*')),
        (os.path.join('share', package_name, 'models'), glob('models/*.pt'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ubuntu',
    maintainer_email='davidfhelmerson@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'depth_perception = perception.depth_perception:main',
            'marble_detector = perception.marble_detection:main',
            'marble_classifier = perception.marble_classifier:main',
            'dp_test = perception.dp_testing:main',
            'image_snapshot = perception.image_snapshot:main',
            'yolo = perception.yolo_node:main'
        ],
    },
)
