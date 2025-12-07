from setuptools import setup

package_name = 'ros2_mqtt_bridge'
project_name = 'ros2-mqtt-bridge' 

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    package_dir={'': 'src'},
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/infrastructure.launch.py']),
        ('share/' + package_name + '/launch', ['launch/car.launch.py']),
        ('share/' + package_name + '/launch', ['launch/car2.launch.py']),
    ],
    install_requires=['setuptools', 'paho-mqtt'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='you@example.com',
    description='ROS 2 -> MQTT Bool bridge',
    license='MIT',
    entry_points={
        'console_scripts': [
            'infrastructure = ros2_mqtt_bridge.infrastructure:main',
            'car = ros2_mqtt_bridge.car:main',
            'car2 = ros2_mqtt_bridge.car2:main',
        ],
    },
)

