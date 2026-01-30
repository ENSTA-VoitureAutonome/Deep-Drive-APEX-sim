from setuptools import setup

package_name = 'voiture_system'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/bringup_sim.launch.py']),
        ('share/' + package_name + '/config', ['config/controllers.yaml']),
        ('share/' + package_name + '/tools', ['tools/test_pipeline.py']),
        ('share/' + package_name + '/urdf', ['urdf/ros2_control.xacro']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='todo',
    maintainer_email='todo@example.com',
    description='High-level controller and simulation bridge for RC car.',
    license='TODO',
    entry_points={
        'console_scripts': [
            'high_level_controller_node = voiture_system.high_level_controller_node:main',
            'vehicle_sim_bridge_node = voiture_system.vehicle_sim_bridge_node:main',
        ],
    },
)
