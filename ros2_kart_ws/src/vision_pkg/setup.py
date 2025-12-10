from setuptools import find_packages, setup
from glob import glob
import os
package_name = 'vision_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['vision_pkg.model.best_ncnn_model', 'vision_pkg.model.best_ncnn_model.*','vision_pkg.best_openvino_model'],include=['vision_pkg','vision_pkg.*']),
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),

        # ncnn ���� share/vision_pkg/model/best_ncnn_model �� ��ġ
        ('share/' + package_name + '/model/best_openvino_model',
         glob('vision_pkg/model/best_openvino_model/*')),
        # (�ʿ��ϸ� .pt�� �Բ�)
        ('share/' + package_name + '/model',
         glob('vision_pkg/model/*.pt')),
    ],
    install_requires=[
        'setuptools',
        'rclpy',
    ],
    include_package_data=True,                      # ← 이 줄!
    package_data={                                  # ← 그리고 이 줄!
    },
    zip_safe=True,
    maintainer='dongjae',
    maintainer_email='dongjae@todo.todo',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'vision_node = vision_pkg.object_detection_node:main'
        ],
    },
)
