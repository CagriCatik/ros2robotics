from setuptools import setup
from glob import glob
import os
package_name = 'rover'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('launch/*.py')),
        (os.path.join('share', package_name), glob('worlds/*')),
        (os.path.join('share', package_name), glob('urdf/*')),
        (os.path.join('lib', package_name), glob('scripts/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='cc',
    maintainer_email='mccatik@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'avoidance = rover.obstacle_avoiding:main',
            'following = rover.follow_wall:main',
        ],
    },
)
