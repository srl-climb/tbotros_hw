from setuptools import setup

package_name = 'serial_manager'

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
    description='Package for managing a serial interface',
    license='Apache 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        'serial_manager = serial_manager.serial_manager_node:main'
        ],
    },
)
