from setuptools import find_packages, setup

package_name = 'sesame_locomotion'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='stregatto',
    maintainer_email='llamanna.luca@gmail.com',
    description='IK verificata e generatore di andatura per il robot Sesame',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'ik_roundtrip = sesame_locomotion.ik_roundtrip:main',
            'reach_surface = sesame_locomotion.reach_surface:main',
        ],
    },
)
