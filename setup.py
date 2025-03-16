from setuptools import find_packages, setup
from glob import glob

package_name = 'final_demo'


other_files = [
	('share/final_demo/launch', glob('launch/*')),
	('share/final_demo/rviz', glob('rviz/*')),
	('share/final_demo/urdf', glob('urdf/*')),
	('share/final_demo/meshes', glob('meshes/*')),
]

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ] + other_files,
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='robot',
    maintainer_email='robot@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        	'board_detector = final_demo.board_detector:main',
        	'fen_detector = final_demo.fen_detector:main',
        	'playgame = final_demo.playgame:main',
            'trajectory = final_demo.trajectory:main',
            'square_detector = final_demo.square_detector:main',
        ],
    },
)
