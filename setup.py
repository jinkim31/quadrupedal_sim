from setuptools import setup

package_name = 'quadrupedal_sim'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name, ['launch/launch.py']),
        ('share/' + package_name + '/urdf', ['urdf/06-flexible.urdf']),
        ('share/' + package_name + '/config', ['config/controller.yaml'])
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='minu',
    maintainer_email='cmw9903@kaist.ac.kr',
    description='Basic Tutorial of ROS2',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'controller = quadrupedal_sim.controller:main'
        ],
    },
)
