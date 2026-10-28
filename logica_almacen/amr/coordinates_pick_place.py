import rclpy
from rclpy.node import Node
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from geometry_msgs.msg import PoseStamped
import math
import time
from tf2_ros import Buffer, TransformListener
from rclpy.duration import Duration

# ================================================================
#  Offset de llegada por estación (metros).
#  El robot se detiene a esta distancia ANTES del ArUco,
#  en la dirección opuesta al frente de la caja.
#  Ajusta según lo que necesite tu brazo xArm5.
# ================================================================
OFFSET_LLEGADA = {
    'pick':    0.20,   # 25cm antes del ArUco de pick
    'classif': 0.15,   # 25cm antes del ArUco de classif
    'home':    0.00,    # home no tiene caja, llega exacto
}

def euler_a_quaternion(yaw):
    return 0.0, 0.0, math.sin(yaw/2.0), math.cos(yaw/2.0)

def make_pose(navigator, x, y, yaw):
    pose = PoseStamped()
    pose.header.frame_id = 'map'
    pose.header.stamp = navigator.get_clock().now().to_msg()
    pose.pose.position.x = x
    pose.pose.position.y = y
    qx, qy, qz, qw = euler_a_quaternion(yaw)
    pose.pose.orientation.x = qx
    pose.pose.orientation.y = qy
    pose.pose.orientation.z = qz
    pose.pose.orientation.w = qw
    return pose

def esperar_nav2(navigator, timeout_seg=30):
    print(f"Esperando action server 'NavigateToPose' (máx {timeout_seg}s)...")
    inicio = time.time()
    while time.time() - inicio < timeout_seg:
        if navigator.nav_to_pose_client.wait_for_server(timeout_sec=1.0):
            print("[OK] Nav2 listo para recibir goals.\n")
            return True
        print("   ... esperando Nav2 (¿presionaste Startup en RViz?)", end='\r')
    print("\n[X] Timeout: Nav2 no respondió en tiempo.")
    return False

def leer_tf_estacion(tf_buffer, nombre):
    """
    Lee posición + orientación del ArUco de la estación.
    Retorna (x, y, yaw) o (None, None, None).
    """
    frame = f'estacion_{nombre}'
    try:
        trans = tf_buffer.lookup_transform(
            'map', frame, rclpy.time.Time(), timeout=Duration(seconds=1.5))
        x = trans.transform.translation.x
        y = trans.transform.translation.y
        q = trans.transform.rotation
        yaw = math.atan2(2*(q.w*q.z + q.x*q.y),
                         1 - 2*(q.y*q.y + q.z*q.z))
        return x, y, yaw
    except Exception:
        return None, None, None

def aplicar_offset(x_aruco, y_aruco, yaw_aruco, offset_m):
    """
    Calcula posición de llegada desplazada offset_m metros
    en dirección +Y del tag (lado opuesto a la caja).
    """
    dx = -math.sin(yaw_aruco) * offset_m
    dy =  math.cos(yaw_aruco) * offset_m
    return x_aruco + dx, y_aruco + dy

def main():
    rclpy.init()
    navigator = BasicNavigator()

    tf_buffer  = Buffer()
    tf_listener = TransformListener(tf_buffer, navigator)

    if not esperar_nav2(navigator, timeout_seg=60):
        print("Asegúrate de haber presionado 'Startup' en RViz.")
        rclpy.shutdown()
        return

    YAW_FIJO = 1.57
    estacion_actual = 'home'

    print("="*40)
    print("  SISTEMA DE DESPACHO - PICKASSO AMR  ")
    print("="*40)
    print("Comandos: home (2), pick (6), classif / place (7), salir")

    while rclpy.ok():
        destino_input = input("\n>> Destino: ").strip().lower()

        if destino_input == 'salir':
            print("Apagando...")
            break

        if destino_input == 'place':
            destino_input = 'classif'

        if destino_input not in ['home', 'pick', 'classif']:
            print(f"[!] Opciones: home, pick, classif (o place)")
            continue

        # Refrescar TF
        for _ in range(15):
            rclpy.spin_once(navigator, timeout_sec=0.05)

        # Tu lógica original de via-point
        puntos_a_visitar = []
        if estacion_actual == 'pick' and destino_input == 'classif':
            print("\n[!] VIA-POINT ACTIVADO: Pick -> Home -> Classif")
            puntos_a_visitar = ['home', 'classif']
        else:
            puntos_a_visitar = [destino_input]

        exito_total = True

        for parada in puntos_a_visitar:
            # Leer posición del ArUco de la estación
            x_aruco, y_aruco, yaw_aruco = leer_tf_estacion(tf_buffer, parada)

            if x_aruco is None:
                print(f"\n[!] ERROR: No se pudo leer '{parada}'. ¿La cámara vio el ArUco?")
                exito_total = False
                break

            # Aplicar offset de llegada
            offset = OFFSET_LLEGADA.get(parada, 0.0)
            if offset > 0.0 and yaw_aruco is not None:
                x_goal, y_goal = aplicar_offset(x_aruco, y_aruco, yaw_aruco, offset)
                print(f"\n[+] {parada.upper()}: ArUco({x_aruco:.2f},{y_aruco:.2f}) "
                      f"→ llegada({x_goal:.2f},{y_goal:.2f}) [offset {offset}m]")
            else:
                x_goal, y_goal = x_aruco, y_aruco
                print(f"\n[+] {parada.upper()}: coordenada ({x_goal:.2f},{y_goal:.2f})")

            goal_pose = make_pose(navigator, x_goal, y_goal, YAW_FIJO)
            print(f"[*] Viajando hacia '{parada.upper()}'")
            navigator.goToPose(goal_pose)

            while not navigator.isTaskComplete():
                feedback = navigator.getFeedback()
                if feedback:
                    print(f"   Distancia restante a {parada.upper()}: "
                          f"{feedback.distance_remaining:.2f} m", end='\r')

            resultado = navigator.getResult()
            print()

            if resultado == TaskResult.SUCCEEDED:
                print(f"[OK] Llegó a {parada.upper()}")
                estacion_actual = parada
            else:
                print(f"[X] Falló el viaje a {parada.upper()}. Abortando ruta.")
                exito_total = False
                break

        if exito_total and len(puntos_a_visitar) > 1:
            print(f"\n[✔] Ruta compuesta hacia {destino_input.upper()} completada!")

    navigator.lifecycleShutdown()
    rclpy.shutdown()

if __name__ == '__main__':
    main()