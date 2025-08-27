from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'rover_system_tests'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='paramesh502',
    maintainer_email='paramesh502@example.com',
    description='ROS 2 system testing package for rover diagnostics',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'test_manager_node = rover_system_tests.test_manager_node:main',
        ],
    },
)
