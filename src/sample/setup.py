import os
from glob import glob
from setuptools import setup

package_name = 'sample'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('launch/*.py')),
        (os.path.join('share', package_name), glob('assets/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='sp',
    maintainer_email='sp@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'odom_subscriber = sample.odom_subscriber:main',
            'build_map = sample.build_map:main',
            'feature_points = sample.feature_points:main',
            'map_talker = sample.map_talker:main',
            'map_talker_copy = sample.map_talker_copy:main',
            'tf_broadcaster = sample.tf_broadcaster:main',
            'zed_odom_tester = sample.zed_odom_tester:main',
            'zed_depth_tester = sample.zed_depth_tester:main',
        ],
    },
)
