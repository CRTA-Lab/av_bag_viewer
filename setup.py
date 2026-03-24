from setuptools import find_packages, setup

package_name = 'bag_viewer'

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
            'bag_viewer = bag_viewer.bag_viewer:main',
        ],
    },
)
