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
                'encoder = raspycar_pkg.encoderR:main',
                'pid = raspycar_pkg.pidR:main',
                'motor = raspycar_pkg.motorR:main',
        ],
    },
)
