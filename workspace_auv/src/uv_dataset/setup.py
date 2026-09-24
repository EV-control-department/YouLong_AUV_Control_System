from setuptools import find_packages, setup

package_name = 'uv_dataset'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/dataset_launch.py']),
    ],
    install_requires=['setuptools', 'numpy', 'opencv-python'],
    zip_safe=True,
    maintainer='origin',
    maintainer_email='origin@example.com',
    description='Raw iceoryx2 dataset recorder',
    license='GPL-3.0',
    entry_points={'console_scripts': ['dataset_recorder = uv_dataset.dataset_recorder:main']},
)
