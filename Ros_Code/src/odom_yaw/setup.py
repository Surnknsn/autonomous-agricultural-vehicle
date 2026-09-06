from setuptools import setup

package_name = 'odom_yaw'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='iai',
    maintainer_email='iai@todo.todo',
    description='Odometry from encoder + Pixhawk yaw',
    license='MIT',
    entry_points={
        'console_scripts': [
            'odom_yaw_node = odom_yaw.odom_yaw_node:main',
        ],
    },
)
