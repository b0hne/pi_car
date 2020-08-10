import os
from glob import glob
from setuptools import setup

package_name = 'pi_car'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('launch/*.launch.py'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='b0hne',
    maintainer_email='erstervonlinks@gmail.com',
    description='rc car using raspberry pi',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'pi_car = pi_car.pi_car:main'
        ],
    },
)
