from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'face_animation_controller'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=[
        'setuptools',
        'requests',
        'flask',
        'flask-cors',
    ],
    zip_safe=True,
    maintainer='kotantu-desktop',
    maintainer_email='Frog7352@icloud.com',
    description='ROS2 node for controlling Sirius face animation via HTTP',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'face_controller = face_animation_controller.face_controller:main',
            'face_http_server = face_animation_controller.http_server:main',
            'expression_publisher = face_animation_controller.expression_publisher:main',
        ],
    },
)
