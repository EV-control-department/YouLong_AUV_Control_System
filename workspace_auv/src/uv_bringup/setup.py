from setuptools import setup

package_name = 'uv_bringup'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', [
            'launch/sim_bringup.py',
            'launch/real_bringup.py',
            'launch/sim_core.py',
            'launch/hil_bringup.py',
        ]),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='origin',
    maintainer_email='origin@example.com',
    description='Launch files for YouLong AUV system bringup',
    license='GPL-3.0',
    tests_require=['pytest'],
    entry_points={},
)
