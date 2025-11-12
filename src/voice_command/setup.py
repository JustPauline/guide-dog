from setuptools import setup
import os
from glob import glob

package_name = 'voice_command'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        # ament index registration
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),

        # install the package.xml
        ('share/' + package_name, ['package.xml']),

        # install launch files if any exist
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*.launch.py')))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='pi',
    maintainer_email='pi@todo.todo',
    description='Voice command processing and translation to robot actions',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'voice_command = voice_command.voice_command:main',
        ],
    },
)
