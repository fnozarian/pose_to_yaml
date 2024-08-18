from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'pose_to_yaml'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*')))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Farzad Nozarian',
    maintainer_email='farzad.nozarian@dfki.de',
    description='A package to dump localized poses from EKF in autoware.universe to YAML files in OpenCood format.',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'pose_to_yaml_node = pose_to_yaml.pose_to_yaml_node:main',
        ],
    },
)
