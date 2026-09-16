from setuptools import setup

package_name = 'uv_bringup'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Explicit list -- keep in sync when adding an entry point.  The
        # simulation presets (sim_bringup / hil_bringup) moved to workspace_sim's
        # uv_sim_bringup package so this file stays free of simulation deps.
        ('share/' + package_name + '/launch', [
            'launch/real_bringup.py',
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
