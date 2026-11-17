from setuptools import find_packages, setup

package_name = 'agv_base_interface'

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
    maintainer='namtruong',
    maintainer_email='namtruong@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
    'console_scripts': [
        'encoder_sim_node = agv_base_interface.encoder_sim_node:main',
        'wheel_odometry_node = agv_base_interface.wheel_odometry_node:main',
        ],
    },
)
