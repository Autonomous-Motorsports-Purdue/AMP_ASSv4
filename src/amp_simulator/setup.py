"""webots_ros2 package setup file."""

from setuptools import setup


package_name = 'amp_simulator'
data_files = []
data_files.append(('share/ament_index/resource_index/packages', ['resource/' + package_name]))
data_files.append(('share/' + package_name + '/launch', ['launch/robot_launch.py']))
data_files.append(('share/' + package_name + '/worlds', [
    'worlds/tesla_world.wbt', 'worlds/.tesla_world.wbproj',
]))
data_files.append(('share/' + package_name + '/resource', [
    'resource/tesla_webots.urdf'
]))
data_files.append(('share/' + package_name, ['package.xml']))


setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=data_files,
    install_requires=['setuptools', 'launch'],
    zip_safe=True,
    maintainer='Cyberbotics',
    maintainer_email='support@cyberbotics.com',
    description='Tesla ROS2 interface for Webots.',
    license='Apache License, Version 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            
        ],
        'launch.frontend.launch_extension': ['launch_ros = launch_ros']
    }
)