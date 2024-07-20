from setuptools import setup

package_name = 'door_navigation'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    py_modules=[],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='you@example.com',
    description='Door navigation package for underwater robot',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'camera_node = door_navigation.camera_node:main',
            'door_detector_node = door_navigation.door_detector_node:main',
            'motor_control_node = door_navigation.motor_control_node:main',
        ],
    },
)
