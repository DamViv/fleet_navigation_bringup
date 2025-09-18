from setuptools import find_packages, setup
from glob import glob
import os

package_name = 'fleet_navigation_bringup'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/' + package_name + '/launch', glob('launch/*.py')),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
        ('share/' + package_name + '/params', glob('params/*.yaml')),
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools', 'PyYAML'],
    zip_safe=True,
    maintainer='Damien Vivet',
    maintainer_email='damien.vivet@isae.fr',
    description='Fleet navigation bringup with traversability map integration',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'goto_remaps_node = fleet_navigation_bringup.goto_remaps:main',
        ],
    },
)
