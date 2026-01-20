from setuptools import setup, find_packages
import os
from glob import glob

package_name = 'litter_bot_perception'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'models'), glob('models/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='hydria',
    maintainer_email='hydria@todo.todo',
    description='Perception package with YOLOv8 for litter detection',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'detection_node = litter_bot_perception.detection_node:main',
            'litter_localizer_node = litter_bot_perception.litter_localizer_node:main',
            'simple_localizer_node = litter_bot_perception.simple_localizer_node:main',
            'color_detection_node = litter_bot_perception.color_detection_node:main',
            'depth_detection_node = litter_bot_perception.depth_detection_node:main',
        ],
    },
)

