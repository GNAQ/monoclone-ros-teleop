# This is used to set up submodules following rospy requirements for monoclone_ros runtime.
# Because ROSpy Runtime Environment modified python system path, include directories, etc.
from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

setup_args = generate_distutils_setup(
    packages=['teleop'],
    package_dir={'': 'src'}
)

setup(**setup_args)
