from setuptools import setup

package_name = 'rasd_gps'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='rasd',
    maintainer_email='rasd@todo.todo',
    description='GPS reader for NEO-7M over UART',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'rasd_gps_node = rasd_gps.rasd_gps_node:main',
        ],
    },
)

