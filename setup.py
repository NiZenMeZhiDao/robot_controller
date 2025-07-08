import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'my_robot_controller'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Add this line to install your launch files
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[py]*'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ares', # Replace with your maintainer name if different
    maintainer_email='ares@todo.todo', # Replace with your email if different
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'robot_controller_node = my_robot_controller.robot_controller_node:main', # Ensure this line exists for your main node
        ],
    },
)