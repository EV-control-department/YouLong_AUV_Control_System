from setuptools import setup

package_name = 'uv_sim_description'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/description.launch.py']),
        ('share/' + package_name + '/urdf', ['urdf/auv_sim.urdf']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='origin',
    maintainer_email='origin@example.com',
    description='Stonefish-only AUV simulation description',
    license='GPL-3.0',
)
