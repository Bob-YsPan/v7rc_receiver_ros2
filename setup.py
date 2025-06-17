from setuptools import find_packages, setup

package_name = 'v7rc_receiver'

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
    maintainer='Bob-YsPan',
    maintainer_email='someone@mail.com',
    description='Use V7RC as the ROS car\'s controller!',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'v7rc_udp_receiver = v7rc_receiver.v7rc_receiver:main_udp',
            'v7rc_uart_receiver = v7rc_receiver.v7rc_receiver:main_uart',
        ],
    },
)
