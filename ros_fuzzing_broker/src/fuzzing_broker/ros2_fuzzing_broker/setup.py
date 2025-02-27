from setuptools import setup

package_name = 'ros2_fuzzing_broker'

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
    maintainer='Lorenz Teply',
    maintainer_email='lorenz@lteply.com',
    description='-',
    license='GNU GPLv3',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "fuzzingbroker_camera = ros2_fuzzing_broker.fuzzingbroker_camera:main",
            "path_subscriber = ros2_fuzzing_broker.path_subscriber:main",
            "awsim_fuzzingbroker_lidar = ros2_fuzzing_broker.awsim_fuzzingbroker_lidar:main"
        ],
    },
    package_dir={'': 'src'},
)
