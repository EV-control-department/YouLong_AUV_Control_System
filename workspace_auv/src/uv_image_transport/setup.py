from setuptools import find_packages, setup

package_name = 'uv_image_transport'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=[
        'setuptools',
        'numpy<1.25; python_version < "3.9"',
        'numpy==1.26.4; python_version >= "3.9"',
    ],
    zip_safe=True,
    maintainer='origin',
    maintainer_email='origin@example.com',
    description='Direct iceoryx2 transport for YouLong AUV camera frames',
    license='GPL-3.0',
    tests_require=['pytest'],
)
