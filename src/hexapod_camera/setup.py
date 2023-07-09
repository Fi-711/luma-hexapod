from setuptools import setup

package_name = 'hexapod_camera'
submodules = "hexapod_camera/submodules"

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
    maintainer_email='ahmedns1@cardiff.ac.uk',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        "camera = hexapod_camera.hexapod_camera:main",
        "detect_ball = hexapod_camera.detect_ball:main",
        "detect_ball_3d = hexapod_camera.detect_ball_3d:main",
        "follow_ball = hexapod_camera.follow_ball:main",
        "detect_person = hexapod_camera.detect_person:main",
        "follow_person = hexapod_camera.follow_person:main",
        "hexapod_object_tracking = hexapod_camera.hexapod_object_tracking:main"
        ],
    },
)
