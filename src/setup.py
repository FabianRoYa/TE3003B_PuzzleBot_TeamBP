from setuptools import find_packages, setup
from glob import glob
import os

package_name = 'blackpearls_nav2_puzzlebot'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
        (os.path.join('share', package_name, 'config'), glob(os.path.join('config', '*.[yma]*'))),
        (os.path.join('share', package_name, 'rviz'), glob(os.path.join('rviz', '*.rviz')))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Fabian Erubiel Rojas Yañez A01706636',
    maintainer_email='A01706636@tec.mx',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'JointStatePublisher = blackpearls_nav2_puzzlebot.joint_state_publisher:main',
            'puzzlebot_sim = blackpearls_nav2_puzzlebot.puzzlebot_sim:main',
            'PointStabilisationController = blackpearls_nav2_puzzlebot.point_stabilisation_controller:main',
            'Localisation = blackpearls_nav2_puzzlebot.localisation:main',
            ''
        ],
    },
)
