from setuptools import find_packages, setup

package_name = 'sensors'

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
    maintainer='mario',
    maintainer_email='emadmario760@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            "ultrasonic_sensor = sensors.ultrasonic_sensor:main",
            "load_cell = sensors.load_cell:main",
            "BMS_sensor = sensors.BMS_sensor:main",
            "battery_LEDS = sensors.battery_LEDS:main",
            "voltage_sensor = sensors.voltage_sensor:main",
            "mpu_sensor = sensors.MPU_sensors:main"
            
        ],
    },
)
