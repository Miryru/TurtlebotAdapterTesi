import os
from setuptools import find_packages, setup
from glob import glob

package_name = 'turtlebot_adapter'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='miriruggi',
    maintainer_email='miriruggi@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'turtlebot_adapter = turtlebot_adapter.turtlebot_adapter:main',
            'turtlebot_adapter2 = turtlebot_adapter.turtlebot_adapter2:main'
            
        ],
    },
)
