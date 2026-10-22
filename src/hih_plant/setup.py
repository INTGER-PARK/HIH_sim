from setuptools import setup
from glob import glob

package_name = 'hih_plant'

setup(
    name=package_name,
    version='0.0.1',
    # ROS package and import package have different names by design:
    # hih_plant/plant is imported as ``plant``.
    packages=['plant'],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/xml', glob('xml/*.xml')),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Ryung',
    maintainer_email='jjungsoo2022@gmail.com',
    description='Cooperative UAV payload simulation',
    license='MIT',
    entry_points={
        'console_scripts': [
            'cooperation_plant = plant.cooperation_plant:main',
            'hih_payload_disturbance = plant.disturbance_nodes:payload_main',
            'hih_wind_disturbance = plant.disturbance_nodes:wind_main',
            'hih_external_wrench_plot = plant.hih_external_wrench_plot:external_wrench_main',
            'hih_topic_plot = plant.hih_external_wrench_plot:main',
        ],
    },
)
