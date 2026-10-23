from setuptools import setup

package_name = 'sl_camera'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml', 'README.md']),
        ('share/' + package_name + '/config', ['config/default.yaml']),
        ('share/' + package_name + '/launch', ['launch/camera.launch.py']),
    ],
    install_requires=['setuptools', 'numpy'],
    zip_safe=True,
    maintainer='origin',
    maintainer_email='origin@example.com',
    description='Independent structured-light monocular MJPEG camera',
    license='GPL-3.0',
    tests_require=['pytest'],
    entry_points={'console_scripts': ['sl_camera = sl_camera.driver:main']},
)
