from setuptools import setup

package_name = 'ur10_publisher'

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
    maintainer='rashmikapu',
    maintainer_email='bhatrashmi.1105@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'ur10_pub = ur10_publisher.ur10_publisher:main',
            'kinematics = ur10_publisher.kinematics:main'
        ],
    },
)
