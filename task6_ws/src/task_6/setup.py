from setuptools import find_packages, setup
from glob import glob
import os

package_name = 'task_6'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),

        (os.path.join('share', package_name), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='athxrva',
    maintainer_email='atharvawadnere3@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "controller_1=task_6.controller_1:main",
            "controller_2=task_6.controller_2:main",
            "controller_3=task_6.controller_3:main",
            "feedback=task_6.feedback:main",
        ],
    },
)
