from setuptools import find_packages, setup

package_name = 'object_locator'

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
    maintainer='grittycan',
    maintainer_email='grittycan@todo.todo',
    description='ombines YOLO and ArUco to find and label objects on the map.',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'text_test = object_locator.test_node:main',
            'aruco_test = object_locator.aruco_test:main',
            'yolo_test = object_locator.yolo_test:main',
            'locator_node = object_locator.locator_node:main'
        ],
    },
)
