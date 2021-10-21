from setuptools import setup

package_name = 'robot'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Martin Jenigar',
    maintainer_email='martin.jenigar@student.tuke.sk',
    description='Robot node - controls robot actions (Move, Rotate...)',
    license='Free to use :)',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'robot = robot.robot:main'
        ],
    },
)
