from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'lab1'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*')),
    ],
    install_requires=['setuptools'],
    #py_modules=['markers'],
    zip_safe=True,
    maintainer='alex',
    maintainer_email='alex@todo.todo',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'nodo_hola = lab1.nodo_hola:main',
            'servo_interactive_publisher = lab1.servo_interactive_publisher:main',
            'servo_to_jointstate = lab1.servo_to_jointstate:main',
	    'newton_raphson= lab1.newton_raphson:main',
	    'control_cinematico = lab1.control_cinematico:main',
        'trayectoria_control_cinematico= lab1.trayectoria_control_cinematico:main',
        'trayectoria_brusca = lab1.trayectoria_brusca:main',
        'trayectoria_interlinear = lab1.trayectoria_interlinear:main',
        'trayectoria_spline = lab1.trayectoria_spline:main',
        'final_sin_retro = lab1.final_sin_retro:main',
        'control_cinematico_2anterio = lab1.Control_cinematico_2anterio:main',
        'trayectoria_brusca_3 = lab1.trayectoria_brusca_3:main',
        'trajectoria_spline_cubico = lab1.trajectoria_spline_cubico:main',
        'trayectoria_control_cinematico_3 = lab1.trayectoria_control_cinematico_3:main',
        ],
    },
)

