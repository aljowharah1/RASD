from setuptools import find_packages, setup

package_name = 'rasd_tf'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),

        # ⭐ Install the launch directory
        ('share/' + package_name + '/launch', [
            'launch/livox_tf.launch.py'
        ]),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='rasd',
    maintainer_email='rasd@todo.todo',
    description='Static transform for Livox MID70',
    license='MIT',
    extras_require={
        'test': ['pytest'],
    },
    entry_points={
        'console_scripts': [
            'livox_tf = rasd_tf.livox_tf:main',
        ],
    },
)

