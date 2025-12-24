import os
import glob
from setuptools import find_packages, setup

package_name = 'youbot_navigation'

def get_data_files():
    data_files = [
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ]

    # install all launch files
    launch_files = glob.glob(os.path.join('launch', '*'))
    if launch_files:
        data_files.append((os.path.join('share', package_name, 'launch'), launch_files))

    # install all config files
    config_files = glob.glob(os.path.join('config', '*'))
    if config_files:
        data_files.append((os.path.join('share', package_name, 'config'), config_files))

    # optional rviz or other resources
    rviz_files = glob.glob(os.path.join('rviz', '*'))
    if rviz_files:
        data_files.append((os.path.join('share', package_name, 'rviz'), rviz_files))

    return data_files

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=get_data_files(),
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='youssef',
    maintainer_email='goldenyouss.art@gmail.com',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        ],
    },
)
