from setuptools import find_packages, setup
from glob import glob

package_name = 'robot_car'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name+'/config', glob("config/*.yaml")),
        ('share/' + package_name+'/config', glob("config/*.rviz")),
        ('share/' + package_name+'/launch', glob("launch/*.py")),
        ('share/' + package_name+'/urdf', glob("urdf/*.xacro")),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='huazhi',
    maintainer_email='huazhi@todo.todo',
    description='TODO: Package description',
    license='MIT',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'test_car_publisher=robot_car.test_car_publisher:main',
            'odom_publisher=robot_car.odom_publisher_node:main',
        ],
    },
)
