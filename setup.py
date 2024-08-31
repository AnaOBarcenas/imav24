from setuptools import find_packages, setup
import os 
from glob import glob

package_name = 'imav24'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('launch/*.launch.py'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='dronkab',
    maintainer_email='perrusquia832@gmail.com',
    description='TODO: Package description',
    license='MIT',
    entry_points={
        'console_scripts': [
            "px4_driver = imav24.px4_driver:main",
            "line_follower = imav24.line_follower:main",
            "camera_pub = imav24.camera_pub:main",
            "aruco_control = imav24.aruco_control:main",
            "joystick_control = imav24.joystick_control:main"
        ],
    },
)
