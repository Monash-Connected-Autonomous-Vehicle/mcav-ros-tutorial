from setuptools import setup
import os
from glob import glob

package_name = 'sim_tut'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.*") )
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='will',
    maintainer_email='will@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
		"control = sim_tut.control:main",
		"sense = sim_tut.sensor:main"
        ],
    },
)
