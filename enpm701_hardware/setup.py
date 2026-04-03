from setuptools import setup

package_name = 'enpm701_hardware'
data_files = []
data_files.append(('share/ament_index/resource_index/packages', ['resource/' + package_name]))
data_files.append(('share/' + package_name, ['package.xml']))
data_files.append(('share/' + package_name + '/launch', ['launch/hardware_launch.py']))

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=data_files,
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Daniel Zinobile',
    maintainer_email='dzinobile@gmail.com',
    description='Hardware interface package for ENPM701 robot',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'colorpicker_node = enpm701_hardware.colorpicker_node:main',
            'teleop_node = enpm701_hardware.teleop_node:main',
            'driver_node = enpm701_hardware.driver_node:main',
],
    },
)
