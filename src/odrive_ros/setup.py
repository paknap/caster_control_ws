from setuptools import setup
from glob import glob

package_name = 'odrive_ros'

setup(
 name=package_name,
 version='0.8.1',
 packages=[package_name],
 data_files=[
     ('share/ament_index/resource_index/packages',
             ['resource/' + package_name]),
     ('share/' + package_name, ['package.xml']),
     ('share/' + package_name + '/launch', glob('launch/*.launch.xml'))
   ],
 install_requires=['setuptools'],
 zip_safe=True,
 maintainer='Josh Marshall, Tecevo',
 maintainer_email='josh.marshall@tecevo.com',
 description='ODrive motor controller driver for ROS',
 license='Modified BSD',
 tests_require=['pytest'],
 entry_points={
     'console_scripts': [
             'odrive_node = odrive_ros.odrive_node:main'
     ],
   },
)