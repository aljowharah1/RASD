from setuptools import setup

package_name = 'rasd_roi_filter'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    package_dir={'': '.'},
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='rasd',
    maintainer_email='rasd@example.com',
    description='ROI filter node',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'roi_node = rasd_roi_filter.roi_node:main',
        ],
    },
)

