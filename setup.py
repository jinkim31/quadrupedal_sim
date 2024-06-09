from setuptools import setup
import os

package_name = 'quadrupedal_sim'

def package_files(data_files, directory_list):
    paths_dict = {}
    for directory in directory_list:
        for (path, directories, filenames) in os.walk(directory):
            for filename in filenames:
                file_path = os.path.join(path, filename)
                install_path = os.path.join('share', package_name, path)
                if install_path in paths_dict.keys():
                    paths_dict[install_path].append(file_path)
                else:
                    paths_dict[install_path] = [file_path]
    for key in paths_dict.keys():
        data_files.append((key, paths_dict[key]))
    return data_files

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=package_files(
        [(os.path.join('share', package_name), ['package.xml', 'launch/launch.py', 'launch/launch.py'])],
        ['world', 'description', 'config']),
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='minu',
    maintainer_email='cmw9903@kaist.ac.kr',
    description='Basic Tutorial of ROS2',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'controller = quadrupedal_sim.controller:main',
            'dataset_gatherer = quadrupedal_sim.dataset_gatherer:main',
        ],
    },
)
