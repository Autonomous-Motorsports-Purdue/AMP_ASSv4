import os
from setuptools import find_packages, setup

package_name = 'amp_lane'

# Iterate through all the files and subdirectories
# to build the data files array
def generate_data_files(share_path, dir):
    data_files = []
    
    for path, _, files in os.walk(dir):
        list_entry = (share_path + path, [os.path.join(path, f) for f in files if not f.startswith('.')])
        data_files.append(list_entry)
    
    return data_files

install_requires=['setuptools', 'mobile_sam @ git+https://github.com/ChaoningZhang/MobileSAM.git']

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # ('share/' + package_name, [package_name + '/lane_follower.py', package_name + '/infer.py'])
    ] + generate_data_files('share/', package_name),
    install_requires=install_requires,
    setup_requires=install_requires,
    zip_safe=True,
    maintainer='lucy',
    maintainer_email='nick@nick.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'lane_follower = amp_lane.lane_follower:main'
        ],
    },
)
