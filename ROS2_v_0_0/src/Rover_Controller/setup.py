from setuptools import setup

package_name = 'Rover_Controller'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='rayv',
    maintainer_email='roypriyanshu09@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "test_node = Rover_Controller.mavlinklistenernode:main",
            "keyboard_node = Rover_Controller.keyboard_publisher:main",
            "control_subscriber = Rover_Controller.control_subscriber:main"
        ],
    },
)
