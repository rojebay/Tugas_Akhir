from setuptools import setup

setup(
    name='my_robot_pkg',
    version='0.0.1',
    packages=['my_robot_pkg'],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/my_robot_pkg']),
        ('share/my_robot_pkg', ['package.xml']),
        ('share/my_robot_pkg/launch', ['launch/sim.launch.py']),  # Tambah path launch
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Nama Anda',
    maintainer_email='you@email.com',
    description='Package untuk robot simulasi Gazebo',
    license='Apache 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'robot_controller = my_robot_pkg.my_robot_node:main',  # Entry point node
        ],
    },
)