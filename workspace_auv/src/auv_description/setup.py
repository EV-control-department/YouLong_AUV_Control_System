from setuptools import setup

package_name = 'auv_description'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/description.launch.py']),
        ('share/' + package_name + '/urdf', ['urdf/auv.urdf']),
        ('share/' + package_name + '/config', [
            'config/real_components.yaml',
        ]),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='origin',
    maintainer_email='origin@example.com',
    description='Real AUV URDF and fixed sensor transforms for the YouLong AUV',
    license='GPL-3.0',
)
