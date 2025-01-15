from setuptools import find_packages, setup

package_name = 'run_motors'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools', 'rclpy', 'dynamixel_sdk'],
    zip_safe=True,
    maintainer='Andy Cheng',
    maintainer_email='andy.cheng@yale.edu',
    description='Package to set position, speed, and torque of motors.',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
		'control_motor = run_motors.control_motor:main'
        ],
    },
)
