from setuptools import find_packages, setup

package_name = 'send_target_joint_degrees'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='yujiro',
    maintainer_email='yujiro5828@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'send_target_joint_degrees = send_target_joint_degrees.send_target_joint_degrees:main'
        ],
    },
)
