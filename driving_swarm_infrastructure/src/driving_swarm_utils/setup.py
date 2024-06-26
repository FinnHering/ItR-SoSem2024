from setuptools import setup

package_name = 'driving_swarm_utils'

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
    maintainer='Sebastian Mai',
    maintainer_email='sebastian.mai@ovgu.de',
    description='utils for DrivingSwarm',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'node = driving_swarm_utils.node:main',
            #'utils = driving_swarm_utils.utils:main',
            'turtlebot_sensor = driving_swarm_utils.turtlebot_sensor:main',
        ],
    },
)
