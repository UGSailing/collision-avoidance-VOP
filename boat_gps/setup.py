from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'boat_gps'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Jasper Van de Velde',
    maintainer_email='jasper.vandevelde@ugent.be',
    description='gps bridge and ntrip client for FLEPOS corrections',
    license='MIT',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'gps_bridge = boat_gps.gps_bridge:main',
            'gps_logger = boat_gps.gps_logger:main',
        ],
    },
)
