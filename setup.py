

import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'logica_almacen'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'maps'), glob('maps/*')), 
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='jerry',
    maintainer_email='jerry@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
        #archivos de Brazo
        'pick_and_place = logica_almacen.brazo.Pick_and_Place_Gripper:main',
        'pick_sin_gripper = logica_almacen.brazo.Pick_Sin_Gripper_Fisico:main',
        'coordenada = logica_almacen.brazo.mover_coordenada:main',
        'llm_cartesian = logica_almacen.brazo.llm_cartesian_controller:main',
        "pick_place = logica_almacen.brazo.pick_and_place:main",

        #archivos de AMR
        'connection_amr_nav2 = logica_almacen.amr.connection_amr_nav2:main',
        'coordinates_pick_place = logica_almacen.amr.coordinates_pick_place:main',
        'comandante = logica_almacen.amr.mision_comandante:main',
        'holonomic_controller_node=logica_almacen.amr.holonomic_controller_node:main',

        
        #vision
         'calibracion = logica_almacen.vision.calibracion:main',
         "amr = logica_almacen.vision.cel9:main",
         "cubos = logica_almacen.vision.pruebaYolo:main",
         "deteccion = logica_almacen.vision.vision_detect:main",
         'amr_vision = logica_almacen.vision.amr_vision:main',
         
        
        ],
    },
)


