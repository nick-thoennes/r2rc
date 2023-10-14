from setuptools import setup

package_name = 'teleoperation_package'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ndt',
    maintainer_email='nick.d.thoennes@gmail.com',
    description='Intermediary between the VESC and the servo controller, and the DS4 controller.',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'teleoperation_node = teleoperation_package.teleoperation_node:main'
        ],
    },
)
