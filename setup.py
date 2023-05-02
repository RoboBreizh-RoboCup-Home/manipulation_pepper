from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup
d = generate_distutils_setup(
    packages=['manipulation_utils','movement','movement_moveit', 'yolov8'],
    package_dir={'': 'scripts'}
)
setup(**d)
