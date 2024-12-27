from setuptools import find_packages, setup

package_name = 'ros2mock'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['ros2cli'],
    zip_safe=True,
    maintainer='max',
    maintainer_email='max@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'ros2cli.command': [
            'mock = ros2mock.command.mock:MockCommand',
        ],
        'ros2cli.extension_point': [
            'ros2mock.verb = ros2mock.verb:VerbExtension',
        ],
        'ros2mock.verb': [
            'service = ros2mock.verb.service:ServiceVerb',
            'action = ros2mock.verb.action:ActionVerb',
        ],
    }
)
