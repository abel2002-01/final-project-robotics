import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'delivery_robot'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        # Package index
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Launch files
        (os.path.join('share', package_name, 'launch'),
            glob('launch/*.py')),
        # Config files
        (os.path.join('share', package_name, 'config'),
            glob('config/*.yaml')),
        # World files
        (os.path.join('share', package_name, 'worlds'),
            glob('worlds/*.sdf')),
        # Model files
        (os.path.join('share', package_name, 'models'),
            glob('models/*.sdf')),
        # Map files
        (os.path.join('share', package_name, 'maps'),
            glob('maps/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='abel',
    maintainer_email='abel@todo.todo',
    description='Indoor Delivery Robot comparing MPPI vs DWB controllers in Nav2',
    license='MIT',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'waypoint_navigator = delivery_robot.waypoint_navigator:main',
            'metrics_logger = delivery_robot.metrics_logger:main',
        ],
    },
)
