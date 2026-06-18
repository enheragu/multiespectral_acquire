from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'multiespectral_acquire_gui'


def _find_data(src_dir, dest_prefix):
    """Recursively collect data files preserving subdirectory structure."""
    result = []
    for dirpath, _, filenames in os.walk(src_dir):
        if not filenames:
            continue
        rel = os.path.relpath(dirpath, src_dir)
        dest = dest_prefix if rel == '.' else os.path.join(dest_prefix, rel)
        result.append((dest, [os.path.join(dirpath, f) for f in filenames]))
    return result


setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.py')),
        *_find_data(
            os.path.join(package_name, 'templates'),
            'share/' + package_name + '/templates'),
        *_find_data(
            os.path.join(package_name, 'static'),
            'share/' + package_name + '/static'),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='quique',
    maintainer_email='enrique.he.ag@gmail.com',
    description='Flask-based GUI for multiespectral cameras',
    license='TODO: License declaration',
    entry_points={
        'console_scripts': [
            'multiespectral_control = multiespectral_acquire_gui.multiespectral_control:main',
        ],
    },
)
