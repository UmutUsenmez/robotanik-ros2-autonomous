from setuptools import find_packages, setup

package_name = 'robotanik_control'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='aziz',
    maintainer_email='azizyvz4@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'location_simulator = robotanik_control.location_simulator_node:main',
            'main_controller = robotanik_control.main_controller_node:main'
        ],
    },
)
