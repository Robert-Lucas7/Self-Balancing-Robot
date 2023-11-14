from setuptools import setup
import os
from glob import glob
package_name = 'sbr_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
        (os.path.join('share', package_name, 'urdf'), glob(os.path.join('urdf', '*.xacro'))),
        (os.path.join('share', package_name, 'world'), glob(os.path.join('world', '*.world'))),
        (os.path.join('share', package_name, 'models'), glob(os.path.join('models', 'robot','*'))),
        (os.path.join('share', package_name, 'meshes'), glob(os.path.join('meshes', '*.stl')))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='rob',
    maintainer_email='rob576821@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
		    'simple_sub = sbr_pkg.simple_subscriber:main',
            'control = sbr_pkg.control:main',
            'SelfBalance = sbr_pkg.pidcontrol:main'
        ],
    },
)
