#!/usr/bin/env python
from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

setup_args = generate_distutils_setup(
    packages=['multiespectral_acquire_gui'],
    package_dir={'': '.'}
)

setup(**setup_args)
