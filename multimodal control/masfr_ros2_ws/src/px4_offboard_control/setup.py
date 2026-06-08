from setuptools import find_packages, setup

package_name = 'px4_offboard_control'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='coolmint417',
    maintainer_email='coolmint417@todo.todo',
    description='ROS 2 package for PX4 offboard control and mission execution',
    license='BSD-3-Clause',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'offboard_control = px4_offboard_control.offboard_control:main',
            'offboard_mission_H = px4_offboard_control.offboard_mission_H:main'
        ],
    },
)
