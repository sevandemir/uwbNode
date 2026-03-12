from setuptools import setup

package_name = 'uwb_mdek1001'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='muhammed-servan',
    maintainer_email='sen@email.com',
    description='MDEK1001 UWB ROS2 Node',
    license='MIT',
    entry_points={
        'console_scripts': [
            'uwb_node = uwb_mdek1001.uwb_node:main',
        ],
    },
)