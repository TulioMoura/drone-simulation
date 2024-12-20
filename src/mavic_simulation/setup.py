"""webots_ros2 package setup file."""

from setuptools import setup


package_name = 'mavic_simulation'
data_files = []
data_files.append(('share/ament_index/resource_index/packages', ['resource/' + package_name]))
data_files.append(('share/' + package_name + '/launch', ['launch/robot_launch.py']))
data_files.append(('share/' + package_name + '/controllers/mavic2pro_navigation' , ['controllers/mavic2pro_navigation/mavic2pro_navigation.py']))
data_files.append(('share/' + package_name + '/worlds', [
    'worlds/mavic_world.wbt', 'worlds/.mavic_world.wbproj',
]))
data_files.append(('share/' + package_name + '/resource', [
    'resource/mavic_webots.urdf'
]))
data_files.append(('share/' + package_name, ['package.xml']))
data_files.append(('share/' + package_name + '/path', ['path/drone_path.json']))


setup(
    name=package_name,
    version='2023.1.1',
    packages=[package_name],
    data_files=data_files,
    install_requires=['setuptools', 'launch'],
    zip_safe=True,
    author='Cyberbotics',
    author_email='support@cyberbotics.com',
    maintainer='Cyberbotics',
    maintainer_email='support@cyberbotics.com',
    keywords=['ROS', 'Webots', 'Robot', 'Simulation', 'Examples'],
    classifiers=[
        'Intended Audience :: Developers',
        'License :: OSI Approved :: Apache Software License',
        'Programming Language :: Python',
        'Topic :: Software Development',
    ],
    description='Mavic 2 Pro robot ROS2 interface for Webots.',
    license='Apache License, Version 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'mavic_driver = mavic_simulation.mavic_driver:main',
            'path_follower = mavic_simulation.path_follower:main'    
        ],
        'launch.frontend.launch_extension': ['launch_ros = launch_ros']
    }
)