from glob import glob
from setuptools import find_packages, setup

package_name = 'depth_processing'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
        ('share/' + package_name + '/web', glob('web/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='jetson-nano-x1',
    maintainer_email='saturnvdt@gmail.com',
    description='ROS 2 nodes for ZED depth viewing and positional tracking',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'depth = depth_processing.depth_processing:main',
            'zed_tracking = depth_processing.zed_positional_tracking:main',
            'zed_base_adapter = depth_processing.zed_base_adapter:main',
            'twist_safety_gate = depth_processing.twist_safety_gate:main',
            'nav_planning_console = depth_processing.nav_planning_console:main',
        ],
    },
)
