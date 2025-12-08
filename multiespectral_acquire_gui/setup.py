from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'multiespectral_acquire_gui'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, package_name, 'templates'), glob('templates/*.html')),
    ],
    install_requires=['setuptools', 'numpy', 'opencv' 'pyyaml', 'multiespectral_acquire', 'sensor_msgs', 'cv_bridge', 'flask', 'flask_socketio'],
    zip_safe=True,
    maintainer='quique',
    maintainer_email='enrique.he.ag@gmail.com',
    description='Flask based GUI to monitor and control multiespectral acquisition from the multispectral_acquire package',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            f'multiespectral_control = {package_name}.multiespectral_control:main',
        ],
    },
)
