from setuptools import setup
from glob import glob
import os


package_name = 'modubot_teleop'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    install_requires=['setuptools','pyserial'],
    zip_safe=True,
    maintainer='levir',
    maintainer_email='levir@example.com',
    description='Bridge /cmd_vel -> ESP32 (V vL vR)',
    license='MIT',
    entry_points={
        'console_scripts': [
            'cmdvel_to_serial = modubot_teleop.cmdvel_to_serial:main',
        ],
    },
    data_files=[
    ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
    ('share/' + package_name, ['package.xml']),
    (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
],

)

