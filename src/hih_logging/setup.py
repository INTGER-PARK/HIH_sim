from setuptools import setup


setup(
    name='hih_logging',
    version='0.0.1',
    packages=['hih_logging'],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/hih_logging']),
        ('share/hih_logging', ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ryung',
    maintainer_email='jjungsoo2022@gmail.com',
    description='Record HandinHand cooperation data as rosbag and CSV',
    license='MIT',
    entry_points={'console_scripts': [
        'hih_record_csv = hih_logging.recorder:main',
        'hih_bag_to_csv = hih_logging.converter:main',
    ]},
)
