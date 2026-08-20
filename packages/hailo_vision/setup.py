import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'hailo_vision'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # 讓系統自動抓取 models 資料夾下的所有 hef 檔
        (os.path.join('share', package_name, 'models'), glob('models/*.hef')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='kj0921',
    maintainer_email='kj0921@todo.todo',
    description='Hailo Vision Package for ROS 2',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'yolo_node = hailo_vision.yolo_node:main',
            'semantic_node = hailo_vision.semantic_node:main',
            'fusion_node = hailo_vision.fusion_node:main',  # 👈 新增這行
        ],
    },
)
