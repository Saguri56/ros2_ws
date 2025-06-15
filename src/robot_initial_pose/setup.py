from setuptools import setup

package_name = 'robot_initial_pose'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='tu_nombre',
    maintainer_email='tu_email@ejemplo.com',
    description='Publica automáticamente la pose inicial para AMCL',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'initial_pose_publisher = robot_initial_pose.initial_pose_publisher:main',
        ],
    },
)
