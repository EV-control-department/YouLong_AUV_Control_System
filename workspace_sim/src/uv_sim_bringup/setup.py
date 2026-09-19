from setuptools import setup

package_name = 'uv_sim_bringup'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', [
            'launch/sim.launch.py',
            'launch/core_sim.launch.py',
            'launch/hil.launch.py',
            'launch/degradation.launch.py',
            'launch/experiment.launch.py',
        ]),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='origin',
    maintainer_email='origin@example.com',
    description='Stonefish and HIL launch orchestration for YouLong AUV',
    license='GPL-3.0',
)
