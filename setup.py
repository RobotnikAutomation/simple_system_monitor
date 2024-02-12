from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

# fetch values from package.xml
setup_args = generate_distutils_setup(
    packages=['simple_system_monitor'],
    package_dir={'': 'src'},
)

setup(**setup_args)