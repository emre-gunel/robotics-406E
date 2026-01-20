from setuptools import setup, find_packages
import os
from glob import glob

package_name = 'litter_bot_navigation'

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
        (os.path.join('share', package_name, 'maps'), glob('maps/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='hydria',
    maintainer_email='hydria@todo.todo',
    description='Navigation package for litter collection robot',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'coverage_planner_node = litter_bot_navigation.coverage_planner_node:main',
        ],
    },
)

