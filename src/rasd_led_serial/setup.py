from setuptools import setup

package_name = 'rasd_led_serial'

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
    description='Serial bridge to Arduino for LED commands',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'led_serial_node = rasd_led_serial.led_serial_node:main',
        ],
    },
)

