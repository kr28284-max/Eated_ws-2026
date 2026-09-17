import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'vision_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (
            os.path.join('share', package_name, 'config'),
            glob(os.path.join('config', '*.yaml')),
        ),
        (
            os.path.join('share', package_name, 'launch'),
            glob(os.path.join('launch', '*.launch.py')),
        ),
    ],
    install_requires=['numpy', 'setuptools', 'ultralytics'],
    zip_safe=True,
    maintainer='fcsl',
    maintainer_email='imsunghy@gmail.com',
    description='On-demand RGB-D object localization for EATED.',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'vision_node = vision_pkg.vision_node:main',
            'vision_node2 = vision_pkg.vision_node2:main',
        ],
    },
)
