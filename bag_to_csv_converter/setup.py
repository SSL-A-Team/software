from setuptools import setup

package_name = 'bag_to_csv_converter'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools', 'flatdict'],
    zip_safe=True,
    maintainer='Collin Avidano',
    maintainer_email='collin.avidano@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'bag_to_csv_converter = bag_to_csv_converter.bag_to_csv_converter:main'
        ],
    },
)
