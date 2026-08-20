import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'smart_delivery_core'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        
        # 🌟 將外部資料夾全部打包，使用 glob('資料夾名稱/*') 抓取裡面所有檔案
        (os.path.join('share', package_name, 'launch'), glob('launch/*')),
        (os.path.join('share', package_name, 'rviz'), glob('rviz/*')),
        (os.path.join('share', package_name, 'maps'), glob('maps/*')),
        (os.path.join('share', package_name, 'config'), glob('config/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='kj0921',
    maintainer_email='todo@todo.com',
    description='Smart delivery core control package',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'cmd_vel_to_serial = smart_delivery_core.cmd_vel_to_serial:main',
            'serial_to_ros = smart_delivery_core.serial_to_ros:main',
            'mecanum_odom_real = smart_delivery_core.mecanum_odom_real:main',
            'serial_bridge = smart_delivery_core.serial_bridge:main',
            'smart_delivery = smart_delivery_core.smart_delivery:main',
        ],
    },
)
