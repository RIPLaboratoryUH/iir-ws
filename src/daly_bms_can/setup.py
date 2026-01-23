from setuptools import find_packages, setup

package_name = 'daly_bms_can'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='riplab',
    maintainer_email='riplab@todo.todo',
    description='Daly BMS CAN reader and ROS 2 publisher',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'daly_bms_pub = daly_bms_can.daly_bms_pub:main',
        ],
    },
)
