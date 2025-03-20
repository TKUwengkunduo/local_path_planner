from setuptools import find_packages, setup
import os

package_name = 'algorithm_local_path_planner'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        # 將 config 資料夾中的 YAML 檔包含進安裝檔案中
        (os.path.join('share', package_name, 'config'), ['config/robot_config.yaml']),
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools', 'pyyaml'],
    zip_safe=True,
    maintainer='iclab',
    maintainer_email='wengkunduo@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'dwa = algorithm_local_path_planner.dwa:main',
            'visualizer = algorithm_local_path_planner.visualizer:main',
        ],
    },
)
