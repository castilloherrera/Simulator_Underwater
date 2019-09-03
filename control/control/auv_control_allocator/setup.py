#***************************************************
# * Title: UUV Simulator
# * Author: The UUV Simulator Authors
# * Date: 2016
# * Availability: https://uuvsimulator.github.io/
#***************************************************

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

setup_args = generate_distutils_setup(
    packages=['auv_actuator_interface'],
    package_dir={'': 'src'},
    requires=['rospy']
)

setup(**setup_args)
