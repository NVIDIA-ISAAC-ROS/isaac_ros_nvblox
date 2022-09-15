from setuptools import setup

package_name = 'nvblox_cpu_gpu_tools'

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
    maintainer='alex',
    maintainer_email='amillane@nvidia.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'cpu_percentage_node = nvblox_cpu_gpu_tools.cpu_percentage_node:main',
            'gpu_percentage_node = nvblox_cpu_gpu_tools.gpu_percentage_node:main'
        ],
    },
)
