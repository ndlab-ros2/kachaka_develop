from setuptools import find_packages, setup

package_name = 'kachaka_feedforward_control'

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
    description='kachaka_feedforward_control',
    license='Apache 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'kachaka_feedforward_control = kachaka_feedforward_control.kachaka_feedforward_control:main',            
        ],
    },
)
