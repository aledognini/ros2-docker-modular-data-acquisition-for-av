import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'pedestrian_visualizer'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Include all launch files
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.py'))),
        # Include all rviz files
        (os.path.join('share', package_name, 'rviz'), glob(os.path.join('rviz', '*.rviz'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='alessandro_dognini',
    maintainer_email='alessandro.dognini@mail.polimi.it',
    description='Pedestrian warning visualizer package',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'pedestrian_warning_node = pedestrian_visualizer.pedestrian_warning_node:main',
        ],
    },
)
