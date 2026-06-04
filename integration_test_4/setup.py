from glob import glob
from setuptools import setup

package_name = 'integration_test_4'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', glob('config/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='root',
    maintainer_email='root@todo.todo',
    description='Clean integration mission package',
    license='TODO',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'mission_orchestrator_node = integration_test_4.mission_orchestrator_node:main',
            'map_republisher_node = integration_test_4.map_republisher_node:main',
            'signal_gate_node = integration_test_4.signal_gate_node:main',
        ],
    },
)
