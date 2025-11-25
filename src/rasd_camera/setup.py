from setuptools import find_packages, setup

package_name = 'rasd_camera'

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
    maintainer='rasd',
    maintainer_email='rasd@todo.todo',
    description='RASD CUDA camera with YOLO',
    license='MIT',
    extras_require={
        'test': ['pytest'],
    },
    entry_points={
	'console_scripts': [
	    'rasd_camera_node = rasd_camera.camera_node:main',
	],

    },
)

