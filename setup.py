from setuptools import find_packages, setup

package_name = "uros_rover_controller"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="george",
    maintainer_email="ngigegeorge023@gmail.com",
    description="TODO: Package description",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "rover_controller = uros_rover_controller.controller:main",
            "rover_data = uros_rover_controller.station:main",
        ],
    },
)
