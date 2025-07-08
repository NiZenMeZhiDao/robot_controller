from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'my_robot_controller'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # 添加 launch 文件
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[py]*'))),
        # 添加 policy.onnx 文件到 share 目录，这样它可以通过 get_package_share_directory 找到
        # 如果你希望通过 get_package_share_directory 获取 ONNX 模型路径，请确保这一行存在
        # 否则，如果你已经硬编码了绝对路径，则不需要
        # (os.path.join('share', package_name), glob('policy.onnx')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='your_name', # 替换为你的名字
    maintainer_email='your_email@example.com', # 替换为你的邮箱
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'robot_controller_node = my_robot_controller.robot_controller_node:main',
            'usb_communicator_node = my_robot_controller.usb_communicator_node:main', # 添加这一行
        ],
    },
)