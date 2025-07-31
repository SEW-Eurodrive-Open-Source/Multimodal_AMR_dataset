from setuptools import setup
import os
from glob import glob

package_name = 'urdf_viewer'

def package_files(source, destination):
    paths = []
    for filepath in glob(os.path.join(source, '**'), recursive=True):
        if os.path.isfile(filepath):
            install_path = os.path.join('share', package_name, destination, os.path.relpath(filepath, source))
            paths.append((os.path.dirname(install_path), [filepath]))
    return paths

data_files = [
    ('share/' + package_name, ['package.xml']),
    ('share/' + package_name + '/resource', ['resource/' + package_name]),
    ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
]

data_files += package_files('launch', 'launch')
data_files += package_files('urdf', 'urdf')
data_files += package_files('meshes', 'meshes')
data_files += package_files('rviz', 'rviz')

setup(
    name=package_name,
    version='0.1.0',
    packages=[],
    data_files=data_files,
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Leon Schönfeld',
    maintainer_email='leon.schoenfeld@sew-eurodrive.de',
    description='ROS 2 URDF Viewer',
    license='MIT',
)
