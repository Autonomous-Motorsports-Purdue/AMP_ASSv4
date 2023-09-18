from setuptools import setup

package_name = 'amp_lane_det'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='lucy',
    maintainer_email='xchenbox@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'lane_follower = amp_simulate.lane_follower:main'
        ],
        'launch.frontend.launch_extension': ['launch_ros = launch_ros']
    },
)
