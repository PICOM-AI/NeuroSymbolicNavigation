from setuptools import setup

package_name = 'semantic_slam'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name, package_name + '.detectors'],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/semantic_slam.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='your_name',
    maintainer_email='your_email@example.com',
    description='Combined semantic SLAM node for TurtleBot 4 with modular detector/fuser.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'semantic_slam_node = semantic_slam.semantic_slam_node:main',
        ],
    },
)
