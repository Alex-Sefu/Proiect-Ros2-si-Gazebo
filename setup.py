from setuptools import setup
import os
from glob import glob

package_name = 'turtlebot_penalty'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'worlds'), glob('worlds/*')),
        (os.path.join('share', package_name, 'models/goal'), glob('models/goal/*')),
        (os.path.join('share', package_name, 'models/obstacle'), glob('models/obstacle/*')),
        (os.path.join('share', package_name, 'models/ball'), glob('models/ball/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='simina',
    maintainer_email='simina@todo.todo',
    description='ROS2 package for a TurtleBot penalty kick simulation.',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'penalty_shooter = turtlebot_penalty.penalty_shooter:main',
        ],
    },
)