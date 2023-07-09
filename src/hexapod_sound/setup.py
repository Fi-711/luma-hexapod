from setuptools import setup

package_name = 'hexapod_sound'

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
    maintainer='pi-ubuntu',
    maintainer_email='ahmedns1@cardiff.ac.uk',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "hexapod_voice_commands = hexapod_sound.hexapod_voice_commands:main",
            "hexapod_sounds = hexapod_sound.hexapod_sounds:main"
        ],
    },
)
