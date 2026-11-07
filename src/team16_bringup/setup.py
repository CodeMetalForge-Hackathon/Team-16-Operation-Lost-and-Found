from setuptools import find_packages, setup

package_name = 'team16_bringup'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Alexander MacConnell',
    maintainer_email='alexander_macconnell@student.uml.edu',
    description='Team 16 ROS 2 Jazzy workspace starter node.',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'hello = team16_bringup.hello:main',
        ],
    },
)
