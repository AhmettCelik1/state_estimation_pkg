from setuptools import setup
import os 
from glob import glob 

package_name = 'state_estimation_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share',package_name,'launch/'),
         glob('launch/*launch.[pxy][yma]*'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ahmet',
    maintainer_email='cahmet644@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "comp_filter = state_estimation_pkg.quaternion_complementary_filter:main",
            "mahony_ahrs = state_estimation_pkg.mahonyAHRS:main",
            "publish_pose = state_estimation_pkg.publish_pose:main",
            "quat2euler = state_estimation_pkg.quat2euler:main",
        ],
    },
)
