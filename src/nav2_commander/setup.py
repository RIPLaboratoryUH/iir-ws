from setuptools import find_packages, setup

package_name = 'nav2_commander'

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
    maintainer_email='lucashorsman@gmail.com',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'lawn_mower = nav2_commander.lawn_mower:main',
            'lawn_mower_heading = nav2_commander.lawn_mower_heading:main',
            'go_home = nav2_commander.go_home:main',
            'lawn_mower_path = nav2_commander.lawn_mower_path:main',
            'waypoint_path = nav2_commander.waypoint_path:main',
        ],
    },
)

