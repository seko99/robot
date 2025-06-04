from setuptools import find_packages, setup

package_name = 'robot_bringup'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/robot.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='seko',
    maintainer_email='seko@example.com',
    description='ROS2 пакет для запуска всех компонентов робота',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [],
    },
)
