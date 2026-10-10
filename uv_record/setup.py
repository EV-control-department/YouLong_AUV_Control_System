from setuptools import setup

package_name = 'uv_record'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools', 'numpy', 'opencv-python'],
    zip_safe=True,
    maintainer='origin',
    maintainer_email='origin@example.com',
    description='Unified raw camera, go2rtc, rosbag and process-log recorder',
    license='GPL-3.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'record = uv_record.recorder:main',
            'recover = uv_record.recover:main',
            'player = uv_record.player:main',
            'export_frames = uv_record.player:export_frames',
            'analyze = uv_record.analyze:main',
        ],
    },
)
