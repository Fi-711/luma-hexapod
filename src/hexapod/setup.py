from setuptools import setup

package_name = 'hexapod'
submodules = "hexapod/submodules"

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name, submodules],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='pi-ubuntu',
    maintainer_email='pi-ubuntu@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "hexapod_movement = hexapod.hexapod_movement:main",
            "hexapod_controller = hexapod.hexapod_controller:main"
        ],
    },
)
