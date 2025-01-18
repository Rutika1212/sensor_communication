from setuptools import find_packages, setup

package_name = 'sensor_communication'

setup(
    name=package_name,
    version='0.0.0',
    packages=['sensor_communication'],
    data_files=[
    
        ('share/' + package_name, ['package.xml']),
    ],

    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='rutika',
    maintainer_email='rutika@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [ 'sensor_communication_node = sensor_communication.sensor_communication_node:main',
        ],
    },
)
