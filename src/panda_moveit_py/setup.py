from setuptools import setup

package_name = 'panda_moveit_py'

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
    maintainer='john',
    maintainer_email='john@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'pose_goal_client = panda_moveit_py.pose_goal_client:main',
            'obstacle_manager = panda_moveit_py.obstacle_manager:main',
            'pose_goal_client_home = panda_moveit_py.pose_goal_client_home:main',
            'pose_grasp = panda_moveit_py.pose_grasp:main',
            'pick_block = panda_moveit_py.pick_block:main',
            'place_block = panda_moveit_py.place_block:main',
        ],
    },
)
