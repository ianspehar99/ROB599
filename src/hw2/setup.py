from setuptools import find_packages, setup

package_name = 'hw2'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools','numpy','rclpy'],
    zip_safe=True,
    maintainer='Ian Spehar',
    maintainer_email='ian.spehar01@gmail.com',
    description='HW2: Topic Practice',
    license='BSD-3-Clause',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        "standard_oscope = hw2.oscope_node:oscope_main",
        "fast_oscope = hw2.oscope_node:fast_wave_main",
        "slow_oscope = hw2.oscope_node:slow_wave_main",
        "limiter_node = hw2.limiter:main"
        ],
    },
)
