from setuptools import setup
from glob import glob
import os

package_name = 'simple_test_system'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='YourName',
    maintainer_email='user@example.com',
    description='A simple system with a tester node and four responder nodes.',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'tester_node = simple_test_system.tester_node:main',
            'responder_node = simple_test_system.responder_node:main',
        ],
    },
)
