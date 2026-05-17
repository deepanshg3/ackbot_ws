from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'ackermann_control'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    
    data_files=[
        # Package index
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),

        # Package manifest
        ('share/' + package_name, ['package.xml']),

        # Launch files
        (os.path.join('share', package_name, 'launch'),
            glob('launch/*.launch.py')),

        # Config files (SLAM yaml etc.)
        (os.path.join('share', package_name, 'config'),
            glob('config/*.yaml')),

        # RViz configs
        (os.path.join('share', package_name, 'config'),
            glob('config/*.rviz')),
            
        # 🔥 Behavior Tree XML files (THIS FIXES THE CRASH)
        (os.path.join('share', package_name, 'config'),
            glob('config/*.xml')),
            
        # MAP FILES
        (os.path.join('share', package_name, 'maps'),
            glob('maps/*')),
    ],

    install_requires=[
        'setuptools',
        'pyserial'
    ],

    zip_safe=True,

    maintainer='deepanshgupta',
    maintainer_email='deepanshgupta@todo.todo',
    description='Ackermann controller and odometry for JetAcker robot',
    license='MIT',

    extras_require={
        'test': ['pytest'],
    },

    entry_points={
        'console_scripts': [
            # Ackermann control node
            'ackermann_controller = ackermann_control.ackermann_controller_node:main',

            # Odometry node
            'odometry_node = ackermann_control.odometry_node:main',
        ],
    },
)
