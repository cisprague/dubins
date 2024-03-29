# Christopher Iliffe Sprague
# sprague@kth.se

from setuptools import setup

setup(
    name='kth-dd2410-planning',
    version='0.0.1',
    author='Christopher Iliffe Sprague',
    author_email='christopher.iliffe.sprague@gmail.com',
    package_dir={'dubins' : 'src'},
    packages=['dubins'],
    url='https://github.com/cisprague/dubins',
    description='Code for the DD2410 planning assignment at KTH.',
    install_requires=['numpy', 'matplotlib', 'scipy', 'jupyter']
)
