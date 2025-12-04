from setuptools import find_packages, setup

package_name = 'can_decoder'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='alessandro_dognini',
    maintainer_email='alessandro.dognini@mail.polimi.it',
    description='Python node to decode CAN messages',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'decoder_node = can_decoder.decoder_node:main',
        ],
    },
)
