from setuptools import find_packages, setup

package_name = 'hw3'

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
    maintainer='Ian Spehar',
    maintainer_email='ian.spehar01@gmail.com',
    description='HW3: Services',
    license='BSD-3-Clause',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "sender = hw3.data_sender:main",
            "receiver = hw3.data_receiver:main",
        ],
    },
)
