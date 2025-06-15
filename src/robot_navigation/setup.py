from setuptools import setup

package_name = 'robot_navigation'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='your_name',
    maintainer_email='your@email.com',
    description='Python Nav2 goal sender',
    license='Apache License 2.0',
    entry_points={
        'console_scripts': [
            'navigate_to_goal = robot_navigation.navigate_to_goal:main',
            'navigate_to_goal_v2 = robot_navigation.navigate_to_goal_v2:main',
        ],
    },
)
