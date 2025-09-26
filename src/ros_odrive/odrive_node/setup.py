from setuptools import setup

package_name = 'odrive_can'

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
    maintainer='RIPLaboratoryUH',
    maintainer_email='your-email@example.com',
    description='ODrive CAN control package',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'keyboard_control = odrive_can.keyboard_control:main'
        ],
    },
    package_dir={'': '.'}
)