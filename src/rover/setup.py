from setuptools import find_packages, setup

package_name = 'rover'

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
    maintainer='ubuntu',
    maintainer_email='ubuntu@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'obstacle_detector = rover.obstacle_detector_node:main',
            'localization_node = rover.localization_node:main',
            'path_planning_node = rover.path_planning_node:main',
            'control_node = rover.control_node:main'
        ],
    },
)
