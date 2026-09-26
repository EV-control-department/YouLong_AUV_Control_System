from glob import glob

from setuptools import find_packages, setup

package_name = 'uv_perception'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/perception_launch.py']),
        ('share/' + package_name + '/weights', glob('weights/*')),
    ],
    install_requires=['setuptools', 'numpy', 'PyYAML'],
    zip_safe=True,
    maintainer='origin',
    maintainer_email='origin@example.com',
    description='iceoryx2 detector, geometry localizer, and track estimator',
    license='GPL-3.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'object_detector = uv_perception.object_detector:main',
            'object_localizer = uv_perception.object_localizer:main',
            'object_estimator = uv_perception.object_estimator:main',
            'perception_gui = uv_perception.perception_gui:main',
        ],
    },
)
