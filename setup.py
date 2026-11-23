import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'yen_bot'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ubuntu',
    maintainer_email='ubuntu@todo.todo',
    description='SO-ARM101 4-DOF arm',
    license='TODO: License declaration',
    extras_require={'test': ['pytest']},
    entry_points={
        'console_scripts': [
            'gui_node = yen_bot.gui.gui_node:main',
            'hardware_node = yen_bot.hardware.hardware_node:main',
            'kinematics_node = yen_bot.kinematics.kinematics_node:main',
            'ros_node = yen_bot.ros.ros_node:main',
            'tools_node = yen_bot.tools.tools_node:main',
        ],
    },
)
