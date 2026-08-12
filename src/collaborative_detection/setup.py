from setuptools import find_packages, setup

package_name = 'collaborative_detection'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', [
            'launch/two_robots.launch.py',
            'launch/experiment.launch.py',
        ]),
        ('share/' + package_name + '/config', [
            'config/params.yaml',
        ]),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Antonio García Alcón',
    maintainer_email='9300toni@gmail.com',
    description='TFM: Detección Colaborativa de Meaconing GNSS mediante Ranging UWB y CUSUM Secuencial',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'gnss_sim_node = collaborative_detection.nodes.gnss_sim_node:main',
            'uwb_sim_node = collaborative_detection.nodes.uwb_sim_node:main',
            'meaconing_injector = collaborative_detection.nodes.meaconing_injector:main',
            'cusum_detector_node = collaborative_detection.nodes.cusum_detector_node:main',
            'robot_mover_node = collaborative_detection.nodes.robot_mover_node:main',
        ],
    },
)
