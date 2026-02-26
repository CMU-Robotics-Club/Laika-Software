from setuptools import find_packages, setup

package_name = 'laika_control'

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
    maintainer='mehul',
    maintainer_email='mehul@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'joint_sin_publisher = laika_control.joint_sin_publisher:main',
            'joint_value_publisher = laika_control.joint_value_publisher:main',
            'joint_ik_publisher = laika_control.joint_ik_publisher:main',
        ],
    },
)
