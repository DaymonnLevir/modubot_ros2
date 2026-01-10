from setuptools import setup
from glob import glob
import os

package_name = 'modubot_joystick'

setup(
    name=package_name,
    version='0.0.1',
    packages=[],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Levir',
    maintainer_email='you@example.com',
    description='Joystick teleop for ModuBot',
    license='MIT',
)

