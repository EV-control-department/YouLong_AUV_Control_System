from setuptools import setup

package_name = 'uv_hm'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/hm_launch.py', 'launch/hardware_launch.py']),
        ('share/' + package_name + '/config', ['config/default.yaml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='origin',
    maintainer_email='origin@example.com',
    description='ZIT6 state adapter and hardware health monitor for YouLong AUV',
    license='GPL-3.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'hw_manager = uv_hm.hw_manager:main',
        ],
    },
)
