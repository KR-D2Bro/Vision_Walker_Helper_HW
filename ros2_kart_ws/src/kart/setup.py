from setuptools import find_packages, setup

package_name = 'kart'

setup(
    name=package_name,
    version='0.0.0',
    #packages=find_packages(exclude=['test']),
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=[
        'setuptools',
        'gatt',
    ],
    zip_safe=True,
    maintainer='dongjae',
    maintainer_email='dongjae@todo.todo',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'ble_peripheral_node = kart.bleNode:main',
        ],
    },
)
