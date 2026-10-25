from setuptools import find_packages, setup

package_name = 'sl_stream'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', ['config/go2rtc.yaml']),
        ('share/' + package_name + '/launch', ['launch/stream.launch.py']),
    ],
    install_requires=['setuptools', 'numpy'],
    tests_require=['pytest'],
    zip_safe=True,
    maintainer='origin',
    maintainer_email='origin@example.com',
    description='On-demand catalog-discovered H.265 camera streams',
    license='GPL-3.0',
    entry_points={
        'console_scripts': [
            'sl_stream = sl_stream.service:main',
            'camera_streamer = sl_stream.camera_streamer:main',
        ],
    },
)
