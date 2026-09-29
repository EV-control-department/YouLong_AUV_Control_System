from setuptools import setup

package_name = 'uv_camera'

data_files = [
    ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
    ('share/' + package_name, ['package.xml', 'GO2RTC.md']),
    ('share/' + package_name + '/launch', [
        'launch/camera_launch.py',
    ]),
    ('share/' + package_name + '/stereos', [
        'stereos/front.yaml',
        'stereos/down.yaml',
    ]),
]

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=data_files,
    # Keep NumPy on the 1.x ABI used by the ROS Python stack.
    install_requires=[
        'setuptools',
        'numpy<1.25; python_version < "3.9"',
        'numpy==1.26.4; python_version >= "3.9"',
    ],
    zip_safe=True,
    maintainer='origin',
    maintainer_email='origin@example.com',
    description='Camera acquisition, CameraInfo, and iceoryx2 raw image publisher',
    license='GPL-3.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'uv_camera = uv_camera.driver:main',
        ],
    },
)
