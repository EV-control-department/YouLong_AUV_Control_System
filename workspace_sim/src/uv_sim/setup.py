from setuptools import setup

package_name = 'uv_sim'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('lib/' + package_name, ['libexec/uv_sim/sim_bridge']),
    ],
    install_requires=['setuptools'],
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
