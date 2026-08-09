from glob import glob
import os

from setuptools import setup


package_name = 'modubot_serial_bridge'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    install_requires=['setuptools', 'pyserial'],
    zip_safe=True,
    maintainer='levir',
    maintainer_email='levir@example.com',
    description='ROS 2 bridge for ModuBot PI firmware and serial telemetry',
    license='MIT',
    entry_points={
        'console_scripts': [
            'cmdvel_to_serial = modubot_serial_bridge.cmdvel_to_serial:main',
            'feedforward_calibration = '
            'modubot_serial_bridge.feedforward_calibration:main',
            'analyze_feedforward_calibration = '
            'modubot_serial_bridge.analyze_feedforward_calibration:main',
            'odometry_trajectory_experiment = '
            'modubot_serial_bridge.odometry_trajectory_experiment:main',
        ],
    },
    data_files=[
        (
            'share/ament_index/resource_index/packages',
            ['resource/' + package_name],
        ),
        (
            'share/' + package_name,
            [
                'package.xml',
                'CALIBRACAO_FEEDFORWARD.md',
                'EXPERIMENTO_TRAJETORIA_ODOMETRIA.md',
            ],
        ),
        (
            os.path.join('share', package_name, 'launch'),
            glob('launch/*.launch.py'),
        ),
        (
            os.path.join('share', package_name, 'config'),
            glob('config/*.yaml'),
        ),
    ],
)
