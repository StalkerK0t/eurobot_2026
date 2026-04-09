from setuptools import find_packages, setup

package_name = 'camera_robot'

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
    maintainer='ubuntu',
    maintainer_email='alusaaa@mail.ru',
    description='TODO: Package description',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'camera_robot_node = camera_robot.camera_robot:main',
            'talker = camera_robot.publisher_member_function:main',
        ],
    },
)

entry_points={
    'console_scripts': [
        'camera_robot_node = camera_robot.camera_robot:main',
        'talker = camera_robot.publisher_member_function:main',
    ],
},