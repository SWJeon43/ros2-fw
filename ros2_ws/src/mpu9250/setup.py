from setuptools import setup

package_name = 'mpu9250'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name,
        f'{package_name}.imusensor',
        f'{package_name}.imusensor.filters',
        f'{package_name}.imusensor.MPU9250'],
    data_files=[
        ('share/ament_index/resource_index/packages', 
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='root',
    maintainer_email='root@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'mpu9250_node = mpu9250.mpu9250_node:main',
            #'tf_broadcaster = mpu9250.tf_broadcaster:main',
        ],
    },
)
