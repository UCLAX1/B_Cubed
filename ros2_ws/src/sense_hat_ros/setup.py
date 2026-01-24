from setuptools import setup

package_name = 'sense_hat_ros'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='x1-raspi5',
    maintainer_email='x1-raspi5@todo.todo',
    description='Sense HAT raw data publisher',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'imu_publisher = sense_hat_ros.imu_publisher:main',
        ],
    },
)
