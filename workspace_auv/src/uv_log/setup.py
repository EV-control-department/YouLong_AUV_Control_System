from setuptools import setup

package_name = 'uv_log'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='origin',
    maintainer_email='origin@example.com',
    description='Crash-resilient ROS2, video and node-log session recorder',
    license='GPL-3.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'record = uv_log.recorder:main',
            'recover = uv_log.recover:main',
            'player = uv_log.player:main',
        ],
    },
)
