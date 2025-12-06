from setuptools import setup
import os
from glob import glob

package_name = 'drone_collision'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Tidak ada launch file di package ini berdasarkan upload Anda
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='User',
    maintainer_email='user@example.com',
    description='Collision monitor package',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # Nama ini harus sesuai dengan yang ada di launch file (executable='...')
            'collision_monitor.py = drone_collision.collision_monitor:main',
        ],
    },
)