from setuptools import find_packages, setup

package_name = 'uv_stream'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', ['config/go2rtc.yaml']),
        ('share/' + package_name + '/launch', ['launch/stream_launch.py']),
    ],
    install_requires=['setuptools', 'numpy', 'opencv-python'],
    zip_safe=True,
    maintainer='origin',
    maintainer_email='origin@example.com',
    description='iceoryx2 to go2rtc display stream adapter',
    license='GPL-3.0',
    entry_points={'console_scripts': ['camera_streamer = uv_stream.camera_streamer:main']},
)
