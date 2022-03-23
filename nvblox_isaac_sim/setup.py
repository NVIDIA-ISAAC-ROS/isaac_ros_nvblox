from setuptools import setup

package_name = 'nvblox_isaac_sim'

setup(
    name=package_name,
    version='0.9.3',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml'])
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Helen Oleynikova',
    maintainer_email='holeynikova@nvidia.com',
    description='Tools for demonstrating nvblox integration with Isaac Sim',
    license='NVIDIA Isaac ROS Software License',
    tests_require=['pytest'],
    entry_points={ },
)
