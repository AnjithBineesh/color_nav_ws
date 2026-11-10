from setuptools import find_packages, setup
from glob import glob
import os

package_name = 'color_navigation'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
            [f for f in glob('launch/*') if os.path.isfile(f)]),
        (os.path.join('share', package_name, 'worlds'),
            [f for f in glob('worlds/*') if os.path.isfile(f)]),
        (os.path.join('share', package_name, 'models'),
            [f for f in glob('models/*') if os.path.isfile(f)]),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='user',
    maintainer_email='anjithbineesh@gmail.com',
    description='Autonomous color detection and navigation package for differential drive robot',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'color_detector = color_navigation.color_detector:main',
            'webcam_detector = color_navigation.webcam_color_detector:main',
            'webcam = color_navigation.webcam_color_detector:main',
            'navigator = color_navigation.navigator:main',
            'spawn_object = color_navigation.spawn_object:main',
            'return_home = color_navigation.return_home:main',
            'home = color_navigation.return_home:main',
            'demo = color_navigation.demo_runner:main',
        ],
    },
)
