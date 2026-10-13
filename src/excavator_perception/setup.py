from setuptools import setup
import os
from glob import glob

package_name = 'excavator_perception'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
         glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    description='Multi-camera merged view and sensor fusion for the AUWO excavator',
    license='MIT',
    entry_points={
        'console_scripts': [
            'merge_clouds = excavator_perception.merge_clouds:main',
        ],
    },
)
