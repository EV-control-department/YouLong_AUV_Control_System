from setuptools import setup

package_name = 'uv_sim'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
    ('share/' + package_name, ['package.xml']),
    ('share/' + package_name + '/launch', ['launch/bridge.launch.py']),
    ('share/' + package_name + '/config/profiles', [
        'config/profiles/sim_dev.yaml',
        'config/profiles/sim_ci.yaml',
        'config/profiles/hil_lab.yaml',
    ]),
    ('lib/' + package_name, ['libexec/uv_sim/sim_bridge']),
    ],
    # Keep the NumPy 1.x ABI used by cv_bridge on both ROS 2 Foxy/Python 3.8
    # and ROS 2 Jazzy/newer Python runtimes.
    # Keep the NumPy 1.x ABI used by cv_bridge on both ROS 2 Foxy/Python 3.8
    # and ROS 2 Jazzy/newer Python runtimes.
    install_requires=[
        'setuptools',
        'numpy<1.25; python_version < "3.9"',
        'numpy==1.26.4; python_version >= "3.9"',
    ],
    zip_safe=True,
    maintainer='origin',
    maintainer_email='origin@example.com',
    description='Simulation-only nodes for YouLong AUV (Stonefish bridge, PID, thruster mixer)',
    license='GPL-3.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'sim_bridge = uv_sim.sim_bridge:main',
        ],
    },
)
