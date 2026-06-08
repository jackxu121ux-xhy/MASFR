from setuptools import find_packages, setup

package_name = 'robot_crawl'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name, ['robot_crawl/trajectory.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='coolmint417',
    maintainer_email='coolmint417@todo.todo',
    description='ROS 2 package for hexapod crawler motion control and trajectory following',
    license='BSD-3-Clause',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'crawl_from_rc = robot_crawl.crawl_from_rc:main',
            'crawl_path_follow = robot_crawl.crawl_path_follow:main'
        ],
    },
)
