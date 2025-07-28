from setuptools import find_packages, setup

package_name = 'actuator_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=[
        'setuptools',
        'rclpy',
        'RPi.GPIO',
    ],
    zip_safe=True,
    maintainer='dongjae',
    maintainer_email='32204045@dankook.ac.kr',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'light_srv = actuator_pkg.light_service:main',
            'buzzer_srv = actuator_pkg.buzzer_service:main',
        ],
    },
)
