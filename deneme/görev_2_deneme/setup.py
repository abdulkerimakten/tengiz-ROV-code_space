from setuptools import setup
import os
from glob import glob

package_name = 'line_follower_package'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='your_name',
    maintainer_email='your_email@example.com',
    description='Package for line following and motor control',
    license='BSD-3-Clause',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'camera_node = line_follower_package.camera_node:main',
            'line_follower_node = line_follower_package.line_follower_node:main',
            'motor_control_node = line_follower_package.motor_control_node:main',
        ],
    },
)
