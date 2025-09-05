from setuptools import find_packages, setup
from glob import glob

package_name = 'kumi_controller'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Launch files
        (f'share/{package_name}/launch', glob('launch/*.py')),

        # URDF / xacro
        (f'share/{package_name}/description', glob('description/*.xacro')),
        
        # Config files (YAML)
        (f'share/{package_name}/config', glob('config/*.yaml')),

        # Worlds (for Gazebo)
        (f'share/{package_name}/worlds', glob('worlds/*')),

        # Controls
        (f'share/{package_name}/kumi_controller', glob('kumi_controller/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='andreas',
    maintainer_email='andreas@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
    'console_scripts': [
        'controller = kumi_controller.kumi_controller:main',
        'com_calculator = kumi_controller.com_calculator:main',
    ]

},

)
