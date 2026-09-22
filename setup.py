from glob import glob

from setuptools import find_packages, setup


package_name = 'uav_energy_inspection'


setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/milestone2.launch.py']),
        ('share/' + package_name + '/worlds', glob('worlds/*.sdf')),
        (
            'share/' + package_name + '/models/inspection_x500',
            glob('models/inspection_x500/*'),
        ),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='aayush1',
    maintainer_email='aayush1@example.com',
    description='ROS 2 nodes for the UAV energy-infrastructure inspection portfolio project.',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'offboard_mission = uav_energy_inspection.offboard_mission:main',
            'waypoint_inspection = uav_energy_inspection.waypoint_inspection:main',
            'sitl_gcs_heartbeat = uav_energy_inspection.sitl_gcs_heartbeat:main',
        ],
    },
)
