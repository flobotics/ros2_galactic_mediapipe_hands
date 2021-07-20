from setuptools import setup

package_name = 'ros2_galactic_mediapipe_hands'

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
    maintainer='ros2',
    maintainer_email='inflo@web.de',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'ros2_galactic_mediapipe_hands = ros2_galactic_mediapipe_hands.ros2_galactic_mediapipe_hands:main'
        ],
        'console_scripts': [
            'ros2_galactic_mediapipe_hands_angle = ros2_galactic_mediapipe_hands.ros2_galactic_mediapipe_hands_angle:main'
        ],
    },
)
