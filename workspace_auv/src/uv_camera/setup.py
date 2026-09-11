import os

from setuptools import setup

package_name = 'uv_camera'

data_files = [
    ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
    ('share/' + package_name, ['package.xml', 'GO2RTC.md']),
    ('share/' + package_name + '/launch', ['launch/perception_launch.py']),
    ('share/' + package_name + '/config', [
        'config/front.npz', 'config/down.npz',
        'config/robotcup_front.npz', 'config/robotcup_down.npz',
        'config/object_localizer_sim.yaml',
    ]),
    ('share/' + package_name + '/config/profiles', [
        'config/profiles/sim_dev.yaml',
        'config/profiles/sim_ci.yaml',
        'config/profiles/hil_lab.yaml',
        'config/profiles/real_default.yaml',
        'config/profiles/real_safe.yaml',
    ]),
]

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=data_files,
    # Keep cv_bridge and NumPy on the same (1.x) ABI used by ROS 2 Jazzy.
    install_requires=['setuptools', 'numpy==1.26.4'],
    zip_safe=True,
    maintainer='origin',
    maintainer_email='origin@example.com',
    description='Camera + AI perception package for YouLong AUV (uv_sensor + uv_ai + object_localizer)',
    license='GPL-3.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'uv_camera = uv_camera.composed:main',
            'object_localizer = uv_camera.object_localizer:main',
            'target_position_gui = uv_camera.target_position_gui:main',
            # Kept as a compatibility entry point; bringup no longer starts it.
            'position = uv_camera.position:main',
        ],
    },
)
