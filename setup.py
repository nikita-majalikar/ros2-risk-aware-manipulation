from setuptools import setup

package_name = 'risk_aware_manipulation'

setup(
    name=package_name,
    version='0.0.0',
    packages=[
        'risk_aware_manipulation',
        'risk_aware_manipulation.perception_uncertainty',
        'risk_aware_manipulation.risk_evaluator',
        'risk_aware_manipulation.moveit_planner',
        'risk_aware_manipulation.execution_monitor',
    ],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ubuntu',
    maintainer_email='ubuntu@todo.todo',
    description='Risk-aware manipulation pipeline in ROS2',
    license='TODO',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'perception_uncertainty_node = risk_aware_manipulation.perception_uncertainty.uncertainty_publisher:main',
            'risk_evaluator_node = risk_aware_manipulation.risk_evaluator.risk_evaluator_node:main',
            'planner_adapter_node = risk_aware_manipulation.moveit_planner.planner_adapter_node:main',
            'execution_monitor_node = risk_aware_manipulation.execution_monitor.execution_monitor_node:main',
            'risk_supervisor_node = risk_aware_manipulation.supervisor.supervisor_node:main',


        ],
    },
)
