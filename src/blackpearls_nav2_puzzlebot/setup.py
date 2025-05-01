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
        (os.path.join('share', package_name, 'rviz'), glob(os.path.join('rviz', '*.rviz'))),
        (os.path.join('share', package_name, 'meshes'), glob(os.path.join('meshes', '*.stl'))), # Odio Ros
        (os.path.join('share', package_name, 'urdf'), glob(os.path.join('urdf', '*.urdf'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Fabian Erubiel Rojas Yañez A01706636\nJose Antonio Miranda Banos ,A01706636\nLuis Fernando Gonzalez Garcia, A01706636\nLuis Fernando Gonzalez Garcia, A01611795',
    maintainer_email='A01706636@tec.mx\nA01611795@tec.mx',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'joint_state_publisher = blackpearls_nav2_puzzlebot.joint_state_publisher:main',
            'puzzlebot_sim = blackpearls_nav2_puzzlebot.puzzlebot_sim:main',
            'point_stabilisation_controller = blackpearls_nav2_puzzlebot.point_stabilisation_controller:main',
            'localisation = blackpearls_nav2_puzzlebot.localisation:main',
            'shapeDrawer = blackpearls_nav2_puzzlebot.shapeDrawer:main',
        ],
    },
)
