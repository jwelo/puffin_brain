from setuptools import find_packages, setup

package_name = 'agent-turtle'

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
    maintainer='jetson',
    maintainer_email='jetson@todo.todo',
    description='turns my turtle INTO A PUFFIN',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'command_publisher_node = agent-turtle.command_publisher_node:main',
            'ollama_publisher_node = agent-turtle.ollama_publisher_node:main',
            'whisper_listener_node = agent-turtle.whisper_listener_node:main',
        ],
    },
)
