import os
from glob import glob
from setuptools import setup

package_name = 'raspycar_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('launch/*.launch.py'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Amerigo',
    maintainer_email='amerigo.sansone.itics@gmail.com',
    description='Differential Drive Speed Control',
    license='Null',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
                'reference = raspycar_pkg.reference:main',
                'encoderR = raspycar_pkg.encoderR:main',
                'encoderL = raspycar_pkg.encoderL:main',
                'pidR = raspycar_pkg.pidR:main',
                'pidL = raspycar_pkg.pidL:main',
                'motorR = raspycar_pkg.motorR:main',
                'motorL = raspycar_pkg.motorL:main',
                'odometry = raspycar_pkg.odometry:main',
                'path_planning = raspycar_pkg.path_planning:main',
        ],
    },
)
