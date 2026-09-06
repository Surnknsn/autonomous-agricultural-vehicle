from setuptools import setup
import os
from glob import glob

package_name = 'my_mavros_launch'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='iai',
    maintainer_email='iai@iai-desktop',
    description='Launch package for MAVROS PX4',
    data_files=[
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
)
