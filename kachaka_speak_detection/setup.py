from setuptools import find_packages, setup

package_name = 'kachaka_speak_detection'

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
    maintainer='Masato Kobayashi',
    maintainer_email='mertcooking@gmail.com',
    description='kachaka_speak_detection',
    license='Apache 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'kachaka_speak_detection = kachaka_speak_detection.kachaka_speak_detection:main'
        ],
    },
)
