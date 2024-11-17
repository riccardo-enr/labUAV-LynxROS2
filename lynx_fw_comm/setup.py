from setuptools import find_packages, setup

package_name = 'lynx_fw_comm'

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
    maintainer='riccardo',
    maintainer_email='riccardo.enrico97@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'k64_sensors_receiver = lynx_fw_comm.k64_sensors_receiver:main',
            'k64_command_sender = lynx_fw_comm.k64_command_sender:main',
            'odometry_sender = lynx_fw_comm.odometry_sender:main',
        ],
    },
)
