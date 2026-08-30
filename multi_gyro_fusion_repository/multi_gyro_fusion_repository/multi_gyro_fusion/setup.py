from setuptools import find_packages, setup
from glob import glob
import os

package_name = 'multi_gyro_fusion'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        (
            'share/ament_index/resource_index/packages',
            ['resource/' + package_name]
        ),
        (
            'share/' + package_name,
            ['package.xml']
        ),
        (
            os.path.join('share', package_name, 'launch'),
            glob('launch/*.launch.py')
        ),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='sharan',
    maintainer_email='sharan@todo.todo',
    description='Eight gyro simulation with ARW, RRW and Markovian noise',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'gyro_publisher = multi_gyro_fusion.gyro_publisher:main',
            'gyro_correlation = multi_gyro_fusion.gyro_correlation:main',
        ],
    },
)
