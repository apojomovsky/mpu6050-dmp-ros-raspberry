from setuptools import setup
from catkin_pkg.python_setup import generate_distutils_setup

setup_args = generate_distutils_setup(
    packages=['mpu6050_dmp_ros'],
    package_dir={'': 'src'},
    scripts=['scripts/imu_publisher']
)

setup(**setup_args)
