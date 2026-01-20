from setuptools import setup, find_packages
import os
from glob import glob

package_name = 'litter_bot_manipulation'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='hydria',
    maintainer_email='hydria@todo.todo',
    description='Manipulation package with MoveIt 2 for litter pickup',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'manipulation_node = litter_bot_manipulation.manipulation_node:main',
            'gripper_controller = litter_bot_manipulation.gripper_controller:main',
        ],
    },
)

