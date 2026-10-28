
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    # 1. Obtener las rutas de nuestro paquete
    pkg_dir = get_package_share_directory('logica_almacen')
    map_file = os.path.join(pkg_dir, 'maps', 'mapa.yaml')
    params_file = os.path.join(pkg_dir, 'config', 'nav2_params.yaml')

    # 2. Nodo: Puente UDP (Comunicación PC -> Raspberry/ESP32)
    # Lo lanzamos primero para que los motores estén listos para escuchar
    udp_bridge_node = Node(
        package='logica_almacen',
        executable='connection_amr_nav2', # Asegúrate de que el nombre coincida con tu setup.py
        name='cmd_vel_bridge',
        output='screen'
    )

    # 3. Nodo: Visión Artificial Perfecta (Los Ojos)
    # Lanza la Realsense/Droidcam y publica de 'odom' a 'base_link'
    vision_node = Node(
        package='logica_almacen',
        executable='amr_vision', # Asegúrate de que el nombre coincida con tu setup.py
        name='global_vision',
        output='screen'
    )

    # 4. Nodo: El Eslabón Estático (map -> odom)
    # Engaña a Nav2 para que confíe en la cámara (Alineando los universos a 0,0,0)
    static_tf_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='map_to_odom',
        arguments=['--x', '0', '--y', '0', '--z', '0', '--yaw', '0', '--pitch', '0', '--roll', '0', '--frame-id', 'map', '--child-frame-id', 'odom'],
        output='screen'
    )

    # 5. Nodo: Lanzar el Ecosistema Nav2
    # Inicia Planificador, Costmaps, Servidor del Mapa, etc.
    nav2_bringup_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('nav2_bringup'), 'launch', 'bringup_launch.py')
        ),
        launch_arguments={
            'map': map_file,
            'params_file': params_file,
            'use_sim_time': 'False',
            'autostart': 'True'
        }.items()
    )

    # 6. Nodo: RViz2 (Visualización en vivo)
    # Abre la interfaz gráfica para ver el carro y darle órdenes
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz_navegacion',
        arguments=['-d', os.path.join(get_package_share_directory('nav2_bringup'), 'rviz', 'nav2_default_view.rviz')],
        output='screen'
    )
    
    # 7. Nodo visión del brazo
    vision_brazo_node = Node(
    	package='logica_almacen',
    	executable='deteccion',
    	name='vision_brazo',
    	output='screen'
    )

    # SECUENCIA DE ENCENDIDO (TimerAction asegura que la cámara encienda antes de Nav2)
    return LaunchDescription([
        udp_bridge_node,
        vision_node,
        static_tf_node,
        vision_brazo_node,
        #pick_place_node,
        TimerAction(
            period=3.0, # Esperamos 3 segundos a que la cámara publique el TF inicial
            actions=[nav2_bringup_launch, rviz_node]
        )
    ])
