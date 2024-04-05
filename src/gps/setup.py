from setuptools import setup
from catkin_pkg.python_setup import generate_distutils_setup

# fetch values from package.xml
setup_args = generate_distutils_setup(
    packages=['gps_driver'],
    package_dir={'': 'python'})

setup(**setup_args)
