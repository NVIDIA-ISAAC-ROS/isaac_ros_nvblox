from setuptools import find_packages, setup

package_name = 'combine_esdf'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Alex Berg',
    maintainer_email='aaberg333@gmail.com',
    description='Combines 2 esdf maps into a single esdf map',
    license='MIT License',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'combine_esdf = combine_esdf.combine_esdf:main',
        ],
    },
)
