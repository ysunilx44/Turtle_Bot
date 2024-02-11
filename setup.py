from setuptools import find_packages, setup

package_name = 'bowser_jr_object_follower'

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
    maintainer='ysunil3',
    maintainer_email='ysunil3@todo.todo',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'find_object_node=bowser_jr_object_follower.find_object_node:main',
        ],
    },
)
