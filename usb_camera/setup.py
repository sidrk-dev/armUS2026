from setuptools import find_packages, setup
import os #added
from glob import glob #added

package_name = 'usb_camera'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')), #added
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='shunt',
    maintainer_email='shuntakam@i.softbank.jp',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        ],
    },
)
