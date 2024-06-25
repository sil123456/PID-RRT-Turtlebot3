from setuptools import find_packages, setup


package_name = 'TurtleBot'

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
            'PID_Controller_LAB3 = TurtleBot.PID_Controller_LAB3:main', # this is for lab 3 
            'Motion_Planner_LAB3 = TurtleBot.Motion_Planner_LAB3:main', # this is for lab 3
            'RRT_node_LAB4 = TurtleBot.RRT_node_LAB4:main', # this is for lab 4
            'Motion_Planner_LAB4 = TurtleBot.Motion_Planner_LAB4:main', # this is for lab 4
            'Motion_Planner = TurtleBot.Motion_Planner:main', # this is for lab 5
            'RRT_node = TurtleBot.RRT_node:main', # this is for lab 5
            'PID_Controller = TurtleBot.PID_Controller:main', # this is for lab 5
        ],
    },
)
