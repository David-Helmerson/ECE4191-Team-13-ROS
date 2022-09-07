from setuptools import setup

package_name = 'navigation'

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
    maintainer='davidhelmerson',
    maintainer_email='davidfhelmerson@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'tentacle_planner = navigation.tentacle_planner:main',
            'stupid_planner = navigation.stupid_planner:main',
            'waypoint_manager = navigation.waypoint_manager:main'
        ],
    },
)
