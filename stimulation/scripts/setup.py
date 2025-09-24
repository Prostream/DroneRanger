from setuptools import setup
import os
from glob import glob

package_name = 'drone_unity_bridge'

setup(
    name=package_name,
    version='0.0.1',
    packages=[],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='justin',
    maintainer_email='justin@example.com',
    description='Unity bridge for drone simulation',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'qos_relay = scripts.qos_relay:main',
        ],
    },
    scripts=['scripts/qos_relay.py'],
)
