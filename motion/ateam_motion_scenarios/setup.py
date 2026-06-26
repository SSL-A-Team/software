from glob import glob

from setuptools import find_packages, setup

package_name = 'ateam_motion_scenarios'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', glob('config/*.json')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='A-Team',
    maintainer_email='ateam@example.com',
    description='Scripted motion scenarios for profiling robot motion performance.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'ball_capture_scenario = ateam_motion_scenarios.ball_capture_scenario:main',
            'catch_scenario = ateam_motion_scenarios.catch_scenario:main',
            'juke_shot_scenario = ateam_motion_scenarios.juke_shot_scenario:main',
            'pass_and_catch_scenario = ateam_motion_scenarios.pass_and_catch_scenario:main',
            'pivot_scenario = ateam_motion_scenarios.pivot_scenario:main',
            'pivot_tune_scenario = ateam_motion_scenarios.pivot_tune_scenario:main',
        ],
    },
)
