from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'bot_ctrl'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ashish',
    maintainer_email='ashishramesh2003@gmail.com',
    description='TODO: Package description',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'path_smoother = bot_ctrl.path_smoother:main',
            'trajectory_generator = bot_ctrl.trajectory_generator:main',
            'trajectory_tracker = bot_ctrl.trajectory_tracker:main',
        ],
    },
)
