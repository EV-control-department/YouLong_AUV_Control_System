from setuptools import setup

package_name = 'uv_mapping'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/map_launch.py']),
        ('share/' + package_name + '/config', ['config/laser_extrinsics.yaml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='origin',
    maintainer_email='origin@example.com',
    description='Line-laser bathymetry: record QA, offline reconstruction, live map',
    license='GPL-3.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'laser_map = uv_mapping.laser_map:main',
            'map_replay = uv_mapping.replay:main',
        ],
    },
)
