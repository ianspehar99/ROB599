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
    maintainer='spehari',
    maintainer_email='spehari@todo.todo',
    description='TODO: Package description',
    license='Apache-2.0',
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
