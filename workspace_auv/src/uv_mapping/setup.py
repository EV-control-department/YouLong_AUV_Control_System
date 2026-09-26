from setuptools import setup

package_name = 'uv_mapping'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml', 'README.md']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='origin',
    maintainer_email='origin@example.com',
    description='[DEPRECATED] Dormant topic shim; use auv_protocol.topics directly',
    license='GPL-3.0',
    tests_require=['pytest'],
)
