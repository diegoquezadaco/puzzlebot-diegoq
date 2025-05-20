import os  
from glob import glob 
from setuptools import find_packages, setup

package_name = 'mobile_robotics'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='diegoq',
    maintainer_email='a01710024@tec.mx',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'test_node = mobile_robotics.test_node:main',
            'move_forward = mobile_robotics.move_forward:main',
            'move_forward_distance = mobile_robotics.move_forward_distance:main', 
            'odometry_node = mobile_robotics.odometry_node:main', 
            'color_tracker = mobile_robotics.color_tracker:main',
            'line_follower_control = mobile_robotics.line_follower_control:main',
            'cv_decision_making = mobile_robotics.cv_decision_making:main',
        ],
    },
)