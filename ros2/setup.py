from setuptools import setup
import os
from glob import glob

package_name = 'lawnmower_control'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('launch/*.py'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='iai',
    maintainer_email='iai@todo.todo',
    description='Autonomous lawnmower control',
    license='MIT',
    entry_points={
        'console_scripts': [
            'lawnmower_node = lawnmower_control.lawnmower_node:main',
            'follow_tracker_node = lawnmower_control.follow_tracker_node:main',
        ],
    },
)

