from setuptools import find_packages, setup
import os
import glob

package_name = 'ecn_turtlebot3x'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob.glob('launch/*.launch.py')),
        ('share/' + package_name + '/config', glob.glob('config/*.yaml')),
        ('share/' + package_name + '/maps', glob.glob('maps/*.yaml')),
        ('share/' + package_name + '/rviz', glob.glob('rviz/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='rohit',
    maintainer_email='rohitpanikarr@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'waffle_odometry = ecn_turtlebot3x.waffle_odometry:main',
            'odom_patch  = ecn_turtlebot3x.odom_covariance_patch:main',
        ],
    },
)
