from pathlib import Path

from setuptools import find_packages, setup

package_name = 'uv_rviz'
mesh_files = sorted(str(path) for path in Path('meshes/youlong').glob('*.obj'))

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml', 'README.md']),
        ('share/' + package_name + '/launch', ['launch/display.launch.py']),
        ('share/' + package_name + '/config', ['config/auv.rviz']),
        ('share/' + package_name + '/meshes/youlong', mesh_files),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='YouLong AUV',
    maintainer_email='origin@example.com',
    description='RViz visualization adapters for the YouLong AUV workspace',
    license='GPL-3.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'visualization_adapter = uv_rviz.visualization_adapter:main',
            'robot_description_adapter = uv_rviz.robot_description_adapter:main',
            'tf_bridge = uv_rviz.tf_bridge:main',
        ],
    },
)
