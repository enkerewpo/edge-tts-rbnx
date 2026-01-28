from setuptools import setup
import os
from glob import glob

package_name = 'edge_tts_rbnx'

# Manifest is at repo root; use relative path (colcon requires relative in data_files)
_manifest_rel = os.path.join('..', '..', 'rbnx_manifest.yaml')
_manifest_abs = os.path.normpath(os.path.join(
    os.path.dirname(os.path.abspath(__file__)), _manifest_rel))
data_files = [
    ('share/ament_index/resource_index/packages',
     ['resource/' + package_name]),
    ('share/' + package_name, ['package.xml']),
]
if os.path.isfile(_manifest_abs):
    data_files.append(('share/' + package_name, [_manifest_rel]))

setup(
    name=package_name,
    version='0.0.1',
    py_modules=[package_name + '_node', 'speak_skill'],
    data_files=data_files,
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='wheatfox',
    maintainer_email='wheatfox17@icloud.com',
    description='Edge TTS ROS2 node for Robonix text-to-speech primitive and speak skill',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'edge_tts_rbnx_node = edge_tts_rbnx_node:main',
            'speak_skill = speak_skill:main',
        ],
    },
)
