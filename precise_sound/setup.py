#!/usr/bin/env python

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

package_info = generate_distutils_setup(
    packages=['precise_sound'],
    package_dir={'': 'src'},
)

setup(**package_info)
