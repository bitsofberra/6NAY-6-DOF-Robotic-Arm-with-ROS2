from setuptools import setup

package_name = 'two_dof_robot'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],  # packages kısmı doğru olmalı
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/display.launch.py']),
        ('share/' + package_name + '/urdf', ['urdf/robot.urdf.xacro']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='yilmaz',
    maintainer_email='yilmaz@example.com',
    description='2-DOF robot example',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'trajectory_publisher = two_dof_robot.trajectory_publisher:main',
            'trajectory_to_target = two_dof_robot.trajectory_to_target:main',

        ],
    },
)
