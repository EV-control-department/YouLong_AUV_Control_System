from setuptools import setup

package_name = 'uv_sim'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', ['config/sim_pid.yaml']),
        ('share/' + package_name + '/launch', ['launch/bridge_launch.py']),
        (
            'share/' + package_name + '/config/extrinsics',
            ['config/extrinsics/xunyun.yaml'],
        ),
    ],
    install_requires=['setuptools', 'PyYAML'],
    zip_safe=True,
    maintainer='origin',
    maintainer_email='origin@example.com',
    description='Simulation-only nodes for YouLong AUV (sensor relays, simulated MCU)',
    license='GPL-3.0',
    tests_require=['pytest'],
    # One console script per node (T5).  There used to be a hand-written
    # libexec/uv_sim/sim_bridge wrapper installed to lib/uv_sim/ as well, which
    # silently *became* the executable: two entry points for one node, kept in
    # sync by memory alone.  test_node_split.py asserts it does not come back.
    entry_points={
        'console_scripts': [
            'odom_sim = uv_sim.odom_sim:main',
            'imu_sim = uv_sim.imu_sim:main',
            'dvl_sim = uv_sim.dvl_sim:main',
            'pressure_sim = uv_sim.pressure_sim:main',
            'nav_sim = uv_sim.nav_sim:main',
            'mcu_sim = uv_sim.mcu_sim:main',
            'camera_relay = uv_sim.camera_relay:main',
            'laser_sim = uv_sim.laser_sim:main',
        ],
    },
)
