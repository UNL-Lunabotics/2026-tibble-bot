import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'description'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join("share", package_name, "urdf"), glob("urdf/*")),
        (os.path.join("share", package_name, "worlds"), glob("worlds/*.sdf")),
        (os.path.join("share", package_name, "worlds/meshes/ksc"), glob("worlds/meshes/ksc/*")),
        (os.path.join("share", package_name, "worlds/meshes/rocks"), glob("worlds/meshes/rocks/*")),
        (os.path.join("share", package_name, "worlds/meshes/terrain"), glob("worlds/meshes/terrain/*")),
        (os.path.join("share", package_name, "worlds/meshes/ucf"), glob("worlds/meshes/ucf/*")),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ubuntu',
    maintainer_email='ubuntu@todo.todo',
    description='TODO: Package description',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
        ],
    },
)
