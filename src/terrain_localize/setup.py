from setuptools import find_packages, setup

package_name = "terrain_localize"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (
            "share/" + package_name + "/maps",
            [
                "maps/synthetic_hill.yaml",
                "maps/synthetic_hill.npy",
                "maps/synthetic_hill.png",
            ],
        ),
        (
            "share/" + package_name + "/launch",
            ["launch/localize.launch.py", "launch/demo.launch.py"],
        ),
        ("share/" + package_name + "/rviz", ["rviz/demo.rviz"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Alexander MacConnell",
    maintainer_email="alexander_macconnell@student.uml.edu",
    description="Match a 3D lidar cloud to a prior heightmap.",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "localizer = terrain_localize.localizer_node:main",
            "check_poses = terrain_localize.check_poses:main",
            "raycast_cloud = terrain_localize.raycast_cloud:main",
            "path_demo = terrain_localize.path_demo:main",
        ],
    },
)
