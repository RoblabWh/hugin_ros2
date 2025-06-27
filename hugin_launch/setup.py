from setuptools import find_packages, setup
from glob import glob

package_name = 'hugin_launch'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*')),
        ('share/' + package_name + '/config', glob('config/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ndigakis',
    maintainer_email='niklas.digakis@studmail.w-hs.de',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'wait_for_bag_end = hugin_launch.helper:wait_for_bag_end',
            'dummy_publisher = hugin_launch.helper:dummy_publisher',
            'reset_kalman_origin = hugin_launch.helper:reset_kalman_origin',
        ],
    },
)
