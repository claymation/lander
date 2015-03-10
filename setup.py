from distutils.core import setup

from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
    packages=["lander"],
    package_dir={"": "src/py"}
)

setup(**d)
