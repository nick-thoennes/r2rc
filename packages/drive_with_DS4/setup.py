from setuptools import setup

package_name = 'drive_with_DS4'

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
    maintainer_email='thoennesn@gmail.com',
    description='Intermediary between the VESC and the servo controller, and the DS4 controller.',
    license='a license to kill >:D',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'driver = drive_with_DS4.driver:main'
        ],
    },
)
