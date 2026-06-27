from setuptools import setup
pkg = 'aisle_flight'
setup(
    name=pkg, version='0.0.1', packages=[pkg],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/'+pkg]),
        ('share/'+pkg, ['package.xml']),
    ],
    install_requires=['setuptools'], zip_safe=True,
    maintainer='xavier', description='MAVROS bridges', license='Proprietary',
    entry_points={'console_scripts': [
        'vio_mavros_relay = aisle_flight.vio_mavros_relay:main',
        'setpoint_bridge = aisle_flight.setpoint_bridge:main',
        'set_ekf_origin = aisle_flight.ekf_origin_setter:main',
        'vio_health_monitor = aisle_flight.vio_health_monitor:main',
    ]},
)
