import os

from setuptools import setup

package_name = 'uv_perception'

# 说明：cv2 标定产物（原 config/*.npz）来自已淘汰的旧摄像头，内参不可信，
# 已随新摄像头一并删除；新摄像头标定完成前不安装任何校准文件。
# 标定后如需分发，再加回 ('share/' + package_name + '/config', [...])。
data_files = [
    ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
    ('share/' + package_name, ['package.xml', 'GO2RTC.md']),
    ('share/' + package_name + '/launch', ['launch/perception_launch.py']),
]

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=data_files,
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='origin',
    maintainer_email='origin@example.com',
    description='Perception package for YouLong AUV',
    license='GPL-3.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'vision = uv_perception.vision:main',
            'position = uv_perception.position:main',
        ],
    },
)
