from setuptools import setup
import os
from glob import glob

package_name = 'edge_tts_rbnx'

setup(
    name=package_name,
    version='0.0.1',
    py_modules=[package_name + '_node'],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='wheatfox',
    maintainer_email='wheatfox17@icloud.com',
    description='Edge TTS ROS2 node for Robonix text-to-speech primitive',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'edge_tts_rbnx_node = edge_tts_rbnx_node:main',
        ],
    },
)
