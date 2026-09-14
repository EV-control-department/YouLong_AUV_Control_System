from setuptools import find_packages, setup


package_name = "smartcar_intro"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
        (f"share/{package_name}/launch", ["launch/status.launch.py"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="student",
    maintainer_email="student@example.com",
    description="ROS 2 publisher and subscriber exercises for the smartcar course",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "status_pub = smartcar_intro.status_pub:main",
            "status_sub = smartcar_intro.status_sub:main",
        ],
    },
)
