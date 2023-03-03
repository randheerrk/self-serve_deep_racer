from setuptools import setup

package_name = 'deep_racer_listener'

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
    maintainer='randheer',
    maintainer_email='randheer.k@thoughtworks.com',
    description='This package contains logic for saving navigation data',
    license='',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'deepracer_listener_node = deep_racer_listener.deepracer_listener:main'
        ],
    },
)
