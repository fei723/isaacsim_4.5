from setuptools import find_packages, setup
import os
package_name = 'joint_controller_pkg'
from glob import glob
setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='shifei',
    maintainer_email='shifei3@xiaomi.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'wheel_control_node = joint_controller_pkg.wheel_control_node:main',
            'wheel_plan_node = joint_controller_pkg.wheel_plan_node:main',
        ],
    },
)
