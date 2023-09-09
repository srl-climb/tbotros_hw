from setuptools import setup

package_name = 'wireless_servo'

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
    maintainer='Simon Harms',
    maintainer_email='harms.simon759@mail.kyutech.jp',
    description='Package to control a hobby servo with an Arduino Nano via Bluetooth BLE',
    license='Apache 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        'wireless_servo = wireless_servo.wireless_servo_node:main',
        'wireless_servo_peripheral = wireless_servo.wireless_servo_peripheral_node:main',
        'wireless_servo_central = wireless_servo.wireless_servo_central_node:main'
        ],
    },
)
