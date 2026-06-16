from setuptools import setup
import os
from glob import glob

package_name = 'originbot_nav2_competition'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'scripts'), glob('scripts/*.sh')),
        (os.path.join('share', package_name, 'sounds'), glob('sounds/*.wav')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Kentiq Flemingaj',
    maintainer_email='elliothahnidu@mail.com',
    description='OriginBot medical competition Nav2 mission package',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'nav2_mission_node = originbot_nav2_competition.nav2_mission_node:main',
            'publish_initial_pose = originbot_nav2_competition.publish_initial_pose:main',
            'nav2_mission_node_hybrid = originbot_nav2_competition.nav2_mission_node_hybrid:main',
        ],
    },
)
