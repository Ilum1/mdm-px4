from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'px4_flight'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Install all launch files
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ros',
    maintainer_email='ros@todo.todo',
    description='TODO: Package description',
    license='MIT',
    #tests_require=['pytest'],
    entry_points={ # 'fly_ring = px4_flight.fly_ring:main'
        'console_scripts': [
            'fly_ring = px4_flight.fly_ring:main'
        ],
    },
)
