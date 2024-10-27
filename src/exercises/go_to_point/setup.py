from setuptools import find_packages, setup

package_name = 'go_to_point'

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
    maintainer='guiphaniel',
    maintainer_email='guilhem.richaud@tuni.fi',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'go_to_point_node = go_to_point.go_to_point_node:main'
        ],
    },
)
