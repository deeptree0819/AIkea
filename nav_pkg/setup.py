from setuptools import find_packages, setup

package_name = 'nav_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    package_data={
        'nav_pkg': ['files/*.json']
    },
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='hj',
    maintainer_email='hj@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'nav_node = nav_pkg.nav_node:main',
            'move_node = nav_pkg.move_node:main',
            'pinky_node = nav_pkg.pinky_node:main',
            'dobot_node = nav_pkg.dobot_node:main',
            'aruco_marker = nav_pkg.aruco_marker:main',
            'dobot_aruco = nav_pkg.dobot_aruco:main',
            'pinky_aruco = nav_pkg.pinky_aruco:main',
            'aruco_marker2D_pub = nav_pkg.aruco_marker2D_pub:main',
            'aruco_dobot_pose = nav_pkg.aruco_dobot_pose:main',
            'dobot_aruco2d = nav_pkg.dobot_aruco2d:main',
            'pinky_aruco2d = nav_pkg.pinky_aruco2d:main',
            'nav_node_client = nav_pkg.nav_node_client:main',
            'aruco_marker2D_test = nav_pkg.aruco_marker2D_test:main',
            'dobot_aruco_test = nav_pkg.dobot_aruco_test:main',
        ],
    },
)

