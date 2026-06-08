from setuptools import find_packages, setup

package_name = 'multimodal_motion'

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
    description='ROS 2 package for coordinating multi-platform motion: UAV flight and hexapod crawling',
    license='BSD-3-Clause',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'manipulation = multimodal_motion.manipulation:main',
            'node_crawling = multimodal_motion.crawl_control:main',
            'node_px4 = multimodal_motion.px4_control:main' 
        ],
    },
)
