from setuptools import find_packages, setup

package_name = 'autoturtle'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='rcpsl',
    maintainer_email='rcpsl@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'my_teleop_node = autoturtle.my_teleop_node:main', # this is for lab 1
            'swim_node = autoturtle.swim_node:main', # this is for lab 2 hw1, trigonometry function
            'two_circle_8shape = autoturtle.two_circle_8shape:main', # this is for lab 2 hw1, 2 circles
            'swim_to_goal = autoturtle.swim_to_goal:main', # this is for lab 2 hw2
            'test = autoturtle.test:main', # this is for lab 2 hw2
            'test2 = autoturtle.test2:main', # this is for lab 2 hw2
            'listener = autoturtle.turtlesim_node:main',
        ],
    },
)
