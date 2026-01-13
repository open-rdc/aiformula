import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'extremum_seeking_mpc'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Issa Omura',
    maintainer_email='issa_omura@jp.honda',
    description='Model Predictive Control using Extremum Seeking Controller',
    license='LGPL-3.0-only',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'extremum_seeking_mpc = extremum_seeking_mpc.extremum_seeking_mpc:main',
        ],
    },
)
