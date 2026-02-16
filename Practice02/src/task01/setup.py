from setuptools import find_packages, setup

package_name = 'task01'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + "/launch", ['launch/task01.launch']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Mariia Fadeeva',
    maintainer_email='mariiya.fadeeva@gmail.com',
    description='Task 01: ROS2 receiver node',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'receiver = task01.receiver:main'
        ],
    },
)
