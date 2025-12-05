from setuptools import find_packages, setup
from glob import glob

package_name = 'robot_car_mecanum'

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
        ('share/' + package_name+'/maps', glob("maps/*.yaml")),
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
            "key_control=robot_car_mecanum.key_control:main",
            "key_control_notime=robot_car_mecanum.key_control_notime:main",
            "send_motor_node=robot_car_mecanum.send_motor_node:main",
            "send_motor_node_key=robot_car_mecanum.send_motor_node_key:main",
        ],
    },
)
