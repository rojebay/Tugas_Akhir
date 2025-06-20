from setuptools import setup, find_packages
import os
from glob import glob

package_name = 'multi_agent'  # UNDERSCORE, bukan hyphen

setup(
    name=package_name,  # Ini akan menggunakan underscore
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'urdf'), glob('urdf/*')),
        (os.path.join('share', package_name, 'worlds'), glob('worlds/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='your_name',
    maintainer_email='your_email@example.com',
    description='Multi-agent robot navigation',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'leader_follower_navigation = multi_agent.leader_follower_navigation:main',
            'apf_navigation = multi_agent.apf_navigation:main', 
            'data_logger = multi_agent.data_logger:main',
            'sensor_streamer = multi_agent.sensor_streamer:main',
        ],
    },
)