import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from geometry_msgs.msg import PoseStamped
from tf2_ros import Buffer, TransformListener
from rclpy.duration import Duration
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

import math
import threading
import time
import json

# ================================================================
#  Offset de llegada por estación (metros) - Lógica de Coordinates
# ================================================================
OFFSET_LLEGADA = {
    'pick':    0.00,   # 20cm antes del ArUco de pick
    'classif': 0.00,   # 20cm antes del ArUco de classif
    'home':    0.00,   # home no tiene caja, llega exacto
}

YAW_FIJO = 1.57


def euler_a_quaternion(yaw):
    return 0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0)


class NodoComandante(Node):
    def __init__(self):
        super().__init__('nodo_comandante_logistico')

        self.pub_bandera = self.create_publisher(String, '/bandera_estacion', 10)
        self.sub_estado = self.create_subscription(String, '/estado_brazo', self.estado_cb, 10)

        # ────────────────────────────────────────────────────────
        # LLM: recibe el color objetivo desde Gemini
        # Espera mensajes tipo:
        # {"color": "RED", "order_id": "123_RED_abc"}
        # También mantiene compatibilidad con mensajes tipo: RED
        # ────────────────────────────────────────────────────────
        target_color_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL
        )

        self.sub_target_color = self.create_subscription(
            String,
            '/target_color',
            self.target_color_cb,
            target_color_qos
        )

        self.estado_brazo = None
        self.estacion_actual = 'home'  # Rastreador de posición

        # Variables de orden desde LLM
        self.orden_activa = False
        self.color_objetivo = None
        self.orden_id_actual = None
        self.ultima_orden_completada = None
        self.colores_validos = ['red', 'blue', 'pink', 'green']

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # --- MEMORIA CONSTANTE DE ARUCOS ---
        self.memoria_estaciones = {}
        self.timer_memoria = self.create_timer(1.5, self._actualizar_memoria)

    def _actualizar_memoria(self):
        """Vigila constantemente el TF y guarda la última posición conocida de cada ArUco"""
        for estacion in ['home', 'pick', 'classif']:
            frame = f'estacion_{estacion}'
            try:
                # Intenta atrapar el TF si está visible en este momento
                trans = self.tf_buffer.lookup_transform('map', frame, rclpy.time.Time())
                x = trans.transform.translation.x
                y = trans.transform.translation.y
                q = trans.transform.rotation
                yaw = math.atan2(
                    2 * (q.w * q.z + q.x * q.y),
                    1 - 2 * (q.y * q.y + q.z * q.z)
                )

                # Actualiza la memoria con las coordenadas más frescas
                self.memoria_estaciones[estacion] = (x, y, yaw)

            except Exception:
                pass  # Si no lo ve, no hace nada, mantiene el último guardado

    def estado_cb(self, msg):
        self.estado_brazo = msg.data.strip()

    # ────────────────────────────────────────────────────────
    # LLM: helpers para recibir color desde /target_color
    # ────────────────────────────────────────────────────────
    def parse_target_color_msg(self, msg):
        raw = msg.data.strip()

        try:
            data = json.loads(raw)
            color = data.get("color", "").strip().lower()
            order_id = data.get("order_id", "").strip()

            if not order_id:
                order_id = f"no_id_{color}"

            return color, order_id

        except Exception:
            # Compatibilidad con mensajes manuales tipo "RED"
            color = raw.lower()
            order_id = f"manual_{color}"
            return color, order_id

    # ────────────────────────────────────────────────────────
    # LLM: callback para recibir color desde /target_color
    # ────────────────────────────────────────────────────────
    def target_color_cb(self, msg):
        color, order_id = self.parse_target_color_msg(msg)

        if color not in self.colores_validos:
            self.get_logger().warn(f"Color no válido recibido desde LLM: {color}")
            return

        if order_id == self.ultima_orden_completada:
            self.get_logger().warn(
                f"Orden ya completada en comandante. Se ignora order_id={order_id}"
            )
            return

        if self.orden_activa:
            self.get_logger().warn(
                f"Ya hay una orden activa ({self.color_objetivo}). "
                f"Se ignora nueva orden: {color} | order_id={order_id}"
            )
            return

        self.color_objetivo = color
        self.orden_id_actual = order_id
        self.orden_activa = True

        self.get_logger().info(
            f"Orden recibida desde LLM: iniciar lote con cubos {color.upper()} | order_id={order_id}"
        )

    def publicar_bandera(self, estacion):
        msg = String()
        msg.data = estacion
        self.pub_bandera.publish(msg)
        self.get_logger().info(f"[!] BANDERA ENVIADA AL BRAZO: '{estacion.upper()}'")

    def esperar_confirmacion_brazo(self, estado_esperado):
        self.estado_brazo = None

        while rclpy.ok() and self.estado_brazo != estado_esperado:
            time.sleep(0.1)

        self.estado_brazo = None

    def leer_posicion_aruco(self, nombre):
        """Retorna las coordenadas desde la memoria constante"""
        if nombre in self.memoria_estaciones:
            return self.memoria_estaciones[nombre]

        return None, None, None


def aplicar_offset(x_aruco, y_aruco, yaw_aruco, offset_m):
    """Calcula posición desplazada en dirección opuesta al frente de la caja"""
    dx = -math.sin(yaw_aruco) * offset_m
    dy =  math.cos(yaw_aruco) * offset_m
    return x_aruco + dx, y_aruco + dy


def navegar_a_dinamico(navigator, nodo, destino):
    """Viaje a un solo punto aplicando offsets"""
    x_aruco, y_aruco, yaw_aruco = nodo.leer_posicion_aruco(destino)

    if x_aruco is None:
        print(f"\n[!] ERROR: No se tiene registro de '{destino}'. ¿La cámara vio el ArUco?")
        return False

    offset = OFFSET_LLEGADA.get(destino, 0.0)

    if offset > 0.0 and yaw_aruco is not None:
        x_goal, y_goal = aplicar_offset(x_aruco, y_aruco, yaw_aruco, offset)
        print(
            f"\n[+] {destino.upper()}: ArUco({x_aruco:.2f},{y_aruco:.2f}) "
            f"→ llegada({x_goal:.2f},{y_goal:.2f}) [offset {offset}m]"
        )
    else:
        x_goal, y_goal = x_aruco, y_aruco
        print(f"\n[+] {destino.upper()}: coordenada ({x_goal:.2f},{y_goal:.2f})")

    goal_pose = PoseStamped()
    goal_pose.header.frame_id = 'map'
    goal_pose.header.stamp = navigator.get_clock().now().to_msg()
    goal_pose.pose.position.x = float(x_goal)
    goal_pose.pose.position.y = float(y_goal)

    qx, qy, qz, qw = euler_a_quaternion(YAW_FIJO)
    goal_pose.pose.orientation.x = qx
    goal_pose.pose.orientation.y = qy
    goal_pose.pose.orientation.z = qz
    goal_pose.pose.orientation.w = qw

    print(f"[*] Viajando hacia '{destino.upper()}'")
    navigator.goToPose(goal_pose)

    while not navigator.isTaskComplete():
        feedback = navigator.getFeedback()

        if feedback:
            print(
                f"   Distancia restante a {destino.upper()}: "
                f"{feedback.distance_remaining:.2f} m",
                end='\r'
            )

    resultado = navigator.getResult()
    print("\n")

    if resultado == TaskResult.SUCCEEDED:
        print(f"[OK] Llegó a {destino.upper()}")
        return True

    else:
        print(f"[X] Falló el viaje a {destino.upper()}. Abortando ruta.")
        return False


def ejecutar_motor_rutas(navigator, nodo, destino_input):
    """Implementa la lógica exacta del via-point de coordinates.py"""
    puntos_a_visitar = []

    if nodo.estacion_actual == 'pick' and destino_input == 'classif':
        print("\n[!] VIA-POINT ACTIVADO: Pick -> Home -> Classif")
        puntos_a_visitar = ['home', 'classif']
    else:
        puntos_a_visitar = [destino_input]

    exito_total = True

    for parada in puntos_a_visitar:
        if not navegar_a_dinamico(navigator, nodo, parada):
            exito_total = False
            break

        # Actualizamos la estación actual paso a paso
        nodo.estacion_actual = parada

    if exito_total and len(puntos_a_visitar) > 1:
        print(f"\n[✔] Ruta compuesta hacia {destino_input.upper()} completada!")

    return exito_total


def main():
    rclpy.init()

    nodo_comunicaciones = NodoComandante()

    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(nodo_comunicaciones)

    thread = threading.Thread(target=executor.spin, daemon=True)
    thread.start()

    navigator = BasicNavigator()

    print("=" * 40)
    print("  SISTEMA MAESTRO DE LOGÍSTICA PICKASSO  ")
    print("=" * 40)

    while rclpy.ok():

        # ────────────────────────────────────────────────────────
        # LLM: esperar orden desde /target_color
        # ────────────────────────────────────────────────────────
        if not nodo_comunicaciones.orden_activa:
            print("\n[Comandante] Esperando orden desde LLM en /target_color...")
            time.sleep(0.5)
            continue

        print("\n" + "=" * 50)
        print(
            f"[Comandante] Nuevo lote iniciado: "
            f"{nodo_comunicaciones.color_objetivo.upper()}"
        )
        print("=" * 50)

        # 1. Ejecutar Viaje a Pick
        if ejecutar_motor_rutas(navigator, nodo_comunicaciones, 'pick'):

            nodo_comunicaciones.publicar_bandera('pick')

            print("\n[Comandante] Esperando a que el xArm5 termine de cargar el carro...")
            nodo_comunicaciones.esperar_confirmacion_brazo('pick_completado')
            print("[Comandante] ¡Carga confirmada por el brazo!")

            # 2. Ejecutar Viaje a Place
            # El motor de rutas detectará que estamos en Pick e insertará 'home' automáticamente
            if ejecutar_motor_rutas(navigator, nodo_comunicaciones, 'classif'):

                nodo_comunicaciones.publicar_bandera('place')

                print("\n[Comandante] Esperando a que el xArm5 termine de descargar...")
                nodo_comunicaciones.esperar_confirmacion_brazo('place_completado')
                print("[Comandante] ¡Descarga confirmada por el brazo!")

                # 3. Retorno a Home final
                ejecutar_motor_rutas(navigator, nodo_comunicaciones, 'home')
                print("\n[✔] Lote finalizado con éxito. Pickasso en reposo.")

        # ────────────────────────────────────────────────────────
        # LLM: liberar orden para aceptar nuevo comando
        # y recordar order_id para evitar loop por TRANSIENT_LOCAL
        # ────────────────────────────────────────────────────────
        nodo_comunicaciones.ultima_orden_completada = nodo_comunicaciones.orden_id_actual
        nodo_comunicaciones.orden_id_actual = None
        nodo_comunicaciones.orden_activa = False
        nodo_comunicaciones.color_objetivo = None

        print("\n[Comandante] Orden liberada. Esperando nuevo color desde LLM.")

    navigator.lifecycleShutdown()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
