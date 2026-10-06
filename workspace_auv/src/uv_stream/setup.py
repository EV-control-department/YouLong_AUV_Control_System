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
    # OpenCV is provided by the ROS image as python3-opencv (cv2). Do not
    # declare pip's opencv-python distribution here: pkg_resources would then
    # reject the ROS-provided cv2 at runtime.
    install_requires=['setuptools', 'numpy'],
    zip_safe=True,
    maintainer='origin',
    maintainer_email='origin@example.com',
    description='iceoryx2 to go2rtc display stream adapter',
    license='GPL-3.0',
    entry_points={'console_scripts': ['camera_streamer = uv_stream.camera_streamer:main']},
)
