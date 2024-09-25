from setuptools import setup, find_packages

package_name = 'iothub_publisher'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Baptiste AMARE',
    maintainer_email='baptiste.amare@sargas.ai',
    description='ROS2 package to send data to Azure IoT Hub',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'send_to_iothub = iothub_publisher.send_to_iothub:main',
        ],
    },
)
