from glob import glob
import os

from setuptools import find_packages, setup

package_name = 'maestro_driver'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
            glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
    ],
    install_requires=['setuptools', 'pyserial'],
    zip_safe=True,
    maintainer='kikai',
    maintainer_email='igarashi.jetson@gmail.com',
    description='ROS2 bridge node for Pololu Maestro servo controller via USB serial',
    license='MIT',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'maestro_driver_node = maestro_driver.maestro_driver_node:main'
        ],
    },
)
