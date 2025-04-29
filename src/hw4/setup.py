from setuptools import find_packages, setup
import os
from glob import glob
package_name = 'hw4'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*'))

    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Ian Spehar',
    maintainer_email='ian.spehar01@gmail.com',
    description='HW4: Actions and more',
    license='BSD-3-Clause',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "oscope = hw4.oscope:main"
        ],
    },
)
