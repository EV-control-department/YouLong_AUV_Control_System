import glob
import os

from setuptools import setup

package_name = 'uv_perception'
repo_dir = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..', '..'))
go2rtc_dir = os.path.join(repo_dir, 'third_party', 'go2rtc')
package_data_files = [
    os.path.relpath(path)
    for path in glob.glob(os.path.join(go2rtc_dir, 'go2rtc'))
]
download_script = os.path.relpath(os.path.join(go2rtc_dir, 'download_go2rtc.sh'))

data_files = [
    ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
    ('share/' + package_name, ['package.xml', 'GO2RTC.md']),
    ('share/' + package_name + '/launch', ['launch/perception_launch.py']),
    ('share/' + package_name + '/config', ['config/front.npz', 'config/down.npz']),
    ('share/' + package_name + '/tools', [download_script]),
]
if package_data_files:
    data_files.append(('share/' + package_name + '/bin', package_data_files))

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=data_files,
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='origin',
    maintainer_email='origin@example.com',
    description='Perception package for YouLong AUV',
    license='GPL-3.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'vision = uv_perception.vision:main',
            'position = uv_perception.position:main',
        ],
    },
)
