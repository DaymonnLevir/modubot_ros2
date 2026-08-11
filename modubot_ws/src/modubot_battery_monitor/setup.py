from setuptools import find_packages, setup

package_name = 'modubot_battery_monitor'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        (
            'share/ament_index/resource_index/packages',
            ['resource/' + package_name],
        ),
        (
            'share/' + package_name,
            ['package.xml'],
        ),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ModuBot',
    maintainer_email='modubot@localhost',
    description='Monitor de tensao e estado da bateria do ModuBot',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'battery_monitor = '
            'modubot_battery_monitor.battery_monitor_node:main',
        ],
    },
)
