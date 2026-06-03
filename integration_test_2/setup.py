import os
from glob import glob
from setuptools import setup

package_name = 'integration_test_2'


def package_files(directory):
    paths = []

    if not os.path.isdir(directory):
        return paths

    for current_path, _, filenames in os.walk(directory):
        files = []

        for filename in filenames:
            files.append(os.path.join(current_path, filename))

        if files:
            install_path = os.path.join('share', package_name, current_path)
            paths.append((install_path, files))

    return paths


data_files = [
    ('share/ament_index/resource_index/packages',
        ['resource/' + package_name]),
    ('share/' + package_name, ['package.xml']),
    ('share/' + package_name + '/config', glob('config/*')),
]

data_files += package_files('fotos')

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=data_files,
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='root',
    maintainer_email='root@todo.todo',
    description='Integration mission package using live A* planning',
    license='TODO',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'integration_mission_runner = integration_test_2.integration_mission_runner:main',
            'signal_gate_node = integration_test_2.signal_gate_node:main',
            'map_republisher_node = integration_test_2.map_republisher_node:main',
            'sift_signal_node = integration_test_2.sift_signal_node:main',
        ],
    },
)
