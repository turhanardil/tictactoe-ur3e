from glob import glob
import os

from setuptools import find_packages, setup

package_name = 'tictactoe_robot'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        (os.path.join('share', package_name), ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'worlds'), glob('worlds/*.sdf')),
        (os.path.join('share', package_name, 'config'), glob('config/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Ardil Turhan',
    maintainer_email='ardilturhan@gmail.com',
    description='Tic-tac-toe pick-and-place simulation with UR3e in Gazebo Harmonic.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'mock_vision = tictactoe_robot.scripts.mock_vision:main',
            'game_ai = tictactoe_robot.scripts.game_ai:main',
        ],
    },
)
