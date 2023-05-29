import os
from glob import glob
from setuptools import setup

package_name = 'ateam_ml'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.xml'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Collin Avidano',
    maintainer_email='collin.avidano@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'bag_to_csv_converter = ateam_ml.bag_to_csv_converter:main',
            'test = bag_to_csv_converter.test:main'
        ],
    },
)
