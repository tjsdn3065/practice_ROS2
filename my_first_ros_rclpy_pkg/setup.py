from setuptools import find_packages, setup

package_name = 'my_first_ros_rclpy_pkg'

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
    maintainer='seonwoo',
    maintainer_email='seonwoo@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={    # 해당 설정을 통해 ros2 run 또는 ros2 launch 명령어로 해당 스크립트를 실행시킬 수 있다.
        'console_scripts': [
          'helloworld_publisher = my_first_ros_rclpy_pkg.helloworld_publisher:main',
          'helloworld_publisher = my_first_ros_rclpy_pkg.helloworld_subscriber:main'
        ],
    },
)
