from setuptools import find_packages, setup

package_name = "lidar_tilt"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (
            "share/" + package_name + "/launch",
            ["launch/sweep.launch.py", "launch/localize.launch.py"],
        ),
        (
            "share/" + package_name + "/rviz",
            ["rviz/sweep.rviz", "rviz/localize.rviz"],
        ),
        ("share/" + package_name + "/maps", ["maps/.gitkeep"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Alexander MacConnell",
    maintainer_email="alexander_macconnell@student.uml.edu",
    description="Step a servo-tilted RPLidar C1 and stack one scan per angle.",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "tilt_sweep = lidar_tilt.tilt_sweep:main",
            "cloud_localize = lidar_tilt.cloud_localize:main",
            "check_register = lidar_tilt.check_register:main",
        ],
    },
)
