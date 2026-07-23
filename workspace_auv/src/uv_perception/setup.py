from setuptools import setup

package_name = 'uv_perception'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/perception_launch.py']),
        ('share/' + package_name + '/config', ['config/front.npz', 'config/down.npz']),
        ('share/' + package_name + '/resource', [
            'resource/box_best.pt',
            'resource/seg_best.pt',
        ]),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='origin',
    maintainer_email='origin@example.com',
    description='Perception package for YouLong AUV',
    license='GPL-3.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'vision = uv_perception.vision:main',
            'position = uv_perception.position:main',
        ],
    },
)
