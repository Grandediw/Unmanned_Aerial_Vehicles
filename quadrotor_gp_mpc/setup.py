from setuptools import find_packages, setup

package_name = 'quadrotor_gp_mpc'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/rviz_visualization.launch.py']),
        ('share/' + package_name + '/launch', ['launch/gazebo_simulation.launch.py']),
        ('share/' + package_name + '/urdf', ['urdf/quadrotor.urdf']),
        ('share/' + package_name + '/worlds', ['worlds/quadrotor_demo.world']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='grandediw',
    maintainer_email='grandediw@todo.todo',
    description='Quadrotor dynamics modeling with Gaussian Process and Model Predictive Control',
    license='MIT',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'quadrotor_dynamics = quadrotor_gp_mpc.quadrotor_dynamics:main',
            'gaussian_process = quadrotor_gp_mpc.gaussian_process:main',
            'mpc_controller = quadrotor_gp_mpc.mpc_controller:main',
            'simple_controller = quadrotor_gp_mpc.simple_controller:main',
            'tf_publisher = quadrotor_gp_mpc.tf_publisher:main',
            'results_visualizer = quadrotor_gp_mpc.results_visualizer:main',
            'main_system = quadrotor_gp_mpc.main:main',
            'demo_system = quadrotor_gp_mpc.demo_system:main',
        ],
    },
)
