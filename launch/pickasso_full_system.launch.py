import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node


def generate_launch_description():

    robot_ip_arg = DeclareLaunchArgument(
        'robot_ip',
        default_value='192.168.1.234',
        description='IP address of the xArm robot'
    )

    robot_ip = LaunchConfiguration('robot_ip')

    # ─────────────────────────────────────────────────────────────
    # 1. AMR / Nav2 / Visión AMR
    # Equivale a:
    # ros2 launch logica_almacen bringup_amr.launch.py
    # ─────────────────────────────────────────────────────────────
    amr_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('logica_almacen'),
                'launch',
                'bringup_amr.launch.py'
            )
        )
    )

    # ─────────────────────────────────────────────────────────────
    # 2. xArm5 real + gripper
    # Equivale a:
    # ros2 launch xarm_planner xarm5_planner_realmove.launch.py robot_ip:=... add_gripper:=true
    # ─────────────────────────────────────────────────────────────
    xarm_realmove = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('xarm_planner'),
                'launch',
                'xarm5_planner_realmove.launch.py'
            )
        ),
        launch_arguments={
            'robot_ip': robot_ip,
            'add_gripper': 'true'
        }.items()
    )

    # ─────────────────────────────────────────────────────────────
    # 3. Pick & Place del brazo
    # Ejecutable correcto:
    # ros2 run logica_almacen pick_place
    # ─────────────────────────────────────────────────────────────
    pick_place_node = Node(
        package='logica_almacen',
        executable='pick_place',
        name='xarm5_pick_place',
        output='screen'
    )

    # ─────────────────────────────────────────────────────────────
    # 4. Comandante AMR
    # Ejecutable correcto:
    # ros2 run logica_almacen comandante
    # ─────────────────────────────────────────────────────────────
    comandante_node = Node(
        package='logica_almacen',
        executable='comandante',
        name='nodo_comandante_logistico',
        output='screen'
    )

    # ─────────────────────────────────────────────────────────────
    # 5. Gemini Bridge
    # Ejecutable:
    # ros2 run gemini_bridge gemini_color_bridge
    # ─────────────────────────────────────────────────────────────
    gemini_bridge_node = Node(
        package='gemini_bridge',
        executable='gemini_color_bridge',
        name='gemini_color_bridge_node',
        output='screen'
    )

    # ─────────────────────────────────────────────────────────────
    # 6. GUI Pickasso
    # Ejecutable:
    # ros2 run gemini_bridge pickasso_gui_qt
    # ─────────────────────────────────────────────────────────────
    pickasso_gui_node = Node(
        package='gemini_bridge',
        executable='pickasso_gui_qt',
        name='pickasso_gui_qt',
        output='screen'
    )

    return LaunchDescription([
        robot_ip_arg,

        # Primero sistema base
        amr_bringup,
        xarm_realmove,

        # Después nodos de lógica
        TimerAction(
            period=5.0,
            actions=[pick_place_node]
        ),

        TimerAction(
            period=7.0,
            actions=[comandante_node]
        ),

        # Después LLM
        TimerAction(
            period=9.0,
            actions=[gemini_bridge_node]
        ),

        TimerAction(
            period=10.0,
            actions=[pickasso_gui_node]
        ),
    ])
