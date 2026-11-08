from setuptools import find_packages, setup

package_name = "terrain_gazebo"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", ["launch/hill.launch.py"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Alexander MacConnell",
    maintainer_email="alexander_macconnell@student.uml.edu",
    description="Gazebo lidar world for the synthetic hill.",
    license="Apache-2.0",
    entry_points={"console_scripts": []},
)
