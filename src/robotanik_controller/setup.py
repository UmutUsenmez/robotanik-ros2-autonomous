from setuptools import find_packages, setup
import os
from glob import glob # Launch dosyalarını bulmak için

package_name = 'robotanik_controller'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Launch dosyasının kurulum yolu
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='aziz',
    maintainer_email='aziz@todo.todo',
    description='Robotanik Ana Kontrol Paketi',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # Çalıştırılabilir düğümlerin tanımı
            'main_controller = robotanik_controller.main_controller:main',
            'heatmap_node = robotanik_controller.heatmap_node:main', # YENİ EKLENEN CANLI HARİTA DÜĞÜMÜ
        ],
    },
)
