from setuptools import find_packages, setup
from glob import glob
import os

package_name = 'demo_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
         glob(os.path.join('launch', '*.xml')))
    ],
    install_requires=['setuptools'],
    maintainer='Noah Zemlin',
    maintainer_email='noah.zemlin@ou.edu',
    description='Demo package for the 2022 SCR Software Challenge',
    license='MIT',
    entry_points={
        'console_scripts': [
            'demo_node = src.demo_node:main'
        ],
    },
)
