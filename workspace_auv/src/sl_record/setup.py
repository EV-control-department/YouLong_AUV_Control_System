from setuptools import setup

package_name = 'sl_record'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml', 'README.md']),
        ('share/' + package_name + '/launch', ['launch/record.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='origin',
    maintainer_email='origin@example.com',
    description='Structured-light JPEG, rosbag and log session recorder',
    license='GPL-3.0',
    tests_require=['pytest'],
    entry_points={'console_scripts': [
        'record = sl_record.recorder:main', 'recover = sl_record.recover:main',
    ]},
)
