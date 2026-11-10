
from setuptools import find_packages, setup
from glob import glob
import os

package_name = 'robot_description'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),

        (os.path.join('share', package_name, 'urdf'),
            [f for f in glob('urdf/*') if os.path.isfile(f)]),

        (os.path.join('share', package_name, 'launch'),
            [f for f in glob('launch/*') if os.path.isfile(f)]),

        (os.path.join('share', package_name, 'worlds'),
            [f for f in glob('worlds/*') if os.path.isfile(f)]),

        (os.path.join('share', package_name, 'config'),
            [f for f in glob('config/*') if os.path.isfile(f)]),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='user',
    maintainer_email='anjithbineesh@gmail.com',
    description='Robot description package for colour detection and differential drive robot',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
        ],
    },
)
