from setuptools import find_packages, setup

package_name = 'av_bag_viewer'

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
    maintainer='branimir',
    maintainer_email='branimir.caran@gmail.com',
    description='ROS2 bag file viewer with Python GUI',
    license='Apache-2.0',
    extras_require={
        'test': ['pytest'],
    },
    entry_points={
        'console_scripts': [
            'av_bag_viewer = av_bag_viewer.av_bag_viewer:main',
            'depth_viewer = av_bag_viewer.depth_viewer:main',
        ],
    },
)
