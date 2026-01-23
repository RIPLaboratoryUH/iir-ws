from setuptools import find_packages, setup

package_name = 'image_saver'

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
    maintainer_email='riplab@example.com',
    description='Saves images from /image_raw topic when space key is pressed',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'image_saver_node = image_saver.image_saver_node:main',
        ],
    },
)
