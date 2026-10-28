#!/usr/bin/env python3
import json
import time
import threading
import subprocess

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from xarm_msgs.srv import PlanSingleStraight, PlanExec


class LLMCartesianController(Node):
    def __init__(self):
        super().__init__('llm_cartesian_controller')

        self.cliente_plan = self.create_client(PlanSingleStraight, '/xarm_straight_plan')
        self.cliente_exec = self.create_client(PlanExec, '/xarm_exec_plan')

        while not self.cliente_plan.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Esperando al servicio /xarm_straight_plan...')

        while not self.cliente_exec.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Esperando al servicio /xarm_exec_plan...')

        self.subscription = self.create_subscription(
            String,
            '/robot_action',
            self.robot_action_callback,
            10
        )

        # Plano Y-Z, con X fija
        self.fixed_x = 0.2

        # HOME físico inicial
        self.home_y = 0.0
        self.home_z = 0.112

        self.current_y = self.home_y
        self.current_z = self.home_z

        # Límites del plano Y-Z en metros
        self.y_min = -0.350   # RIGHT
        self.y_max = 0.350    # LEFT
        self.z_min = 0.110    # DOWN
        self.z_max = 0.540    # UP

        self.busy = False
        self.lock = threading.Lock()

        self.get_logger().info('Nodo LLM Cartesian Controller iniciado.')
        self.get_logger().info(
            f'Modo Straight Plan: X fijo={self.fixed_x}, '
            f'HOME=[{self.fixed_x}, {self.home_y}, {self.home_z}]'
        )
        self.get_logger().info(
            f'Límites: Y=[{self.y_min}, {self.y_max}], '
            f'Z=[{self.z_min}, {self.z_max}]'
        )

    def activar_controller(self):
        try:
            result = subprocess.run(
                [
                    'ros2', 'control', 'set_controller_state',
                    'xarm5_traj_controller', 'active'
                ],
                capture_output=True,
                text=True,
                timeout=8
            )

            if result.returncode == 0:
                self.get_logger().info('xarm5_traj_controller activo/verificado.')
                return True

            self.get_logger().warn(
                f'No se pudo activar controller: {result.stderr.strip()}'
            )
            return False

        except Exception as e:
            self.get_logger().error(f'Error activando controller: {e}')
            return False

    def limpiar_error_robot(self):
        try:
            subprocess.run(
                ['ros2', 'service', 'call', '/xarm/clear_error', 'xarm_msgs/srv/Call', '{}'],
                capture_output=True,
                text=True,
                timeout=5
            )
            subprocess.run(
                ['ros2', 'service', 'call', '/xarm/set_state', 'xarm_msgs/srv/SetInt16', '{data: 0}'],
                capture_output=True,
                text=True,
                timeout=5
            )
        except Exception as e:
            self.get_logger().warn(f'No se pudo limpiar error/set_state: {e}')

    def robot_action_callback(self, msg):
        with self.lock:
            if self.busy:
                self.get_logger().warn('Robot ocupado. Ignorando nuevo comando.')
                return
            self.busy = True

        try:
            data = json.loads(msg.data)
        except json.JSONDecodeError:
            self.get_logger().error(f'JSON inválido recibido: {msg.data}')
            with self.lock:
                self.busy = False
            return

        self.get_logger().info(f'Comando recibido: {data}')

        command = data.get('command')
        direction = data.get('direction')

        target_y = self.current_y
        target_z = self.current_z

        if command == 'INVALID':
            self.get_logger().warn(f'Comando inválido ignorado: {data.get("reason")}')
            with self.lock:
                self.busy = False
            return

        elif command == 'HOME':
            target_y = self.home_y
            target_z = self.home_z
            self.get_logger().info('Comando HOME recibido.')

        elif command == 'STOP':
            self.get_logger().warn('STOP recibido. No se ejecuta nuevo movimiento.')
            with self.lock:
                self.busy = False
            return

        elif command == 'MOVE':
            if direction == 'UP':
                target_z = self.z_max
            elif direction == 'DOWN':
                target_z = self.z_min
            elif direction == 'LEFT':
                target_y = self.y_max
            elif direction == 'RIGHT':
                target_y = self.y_min
            else:
                self.get_logger().warn(f'Dirección no soportada: {direction}')
                with self.lock:
                    self.busy = False
                return
        else:
            self.get_logger().warn(f'Comando no soportado: {command}')
            with self.lock:
                self.busy = False
            return

        target_y = max(self.y_min, min(target_y, self.y_max))
        target_z = max(self.z_min, min(target_z, self.z_max))

        self.get_logger().info(
            f'Objetivo calculado: X={self.fixed_x}, Y={target_y}, Z={target_z}'
        )

        worker = threading.Thread(
            target=self.execute_motion_sequence,
            args=(self.fixed_x, target_y, target_z),
            daemon=True
        )
        worker.start()

    def wait_future_in_thread(self, future, timeout_sec=45.0):
        start = time.time()
        while not future.done():
            if time.time() - start > timeout_sec:
                return None
            time.sleep(0.05)
        return future.result()

    def execute_motion_sequence(self, x, y, z):
        try:
            self.limpiar_error_robot()
            self.activar_controller()

            req = PlanSingleStraight.Request()

            req.target.position.x = x
            req.target.position.y = y
            req.target.position.z = z

            # Misma orientación que venías usando con MoveIt
            req.target.orientation.x = 1.0
            req.target.orientation.y = 0.0
            req.target.orientation.z = 0.0
            req.target.orientation.w = 0.0

            self.get_logger().info(
                f'Calculando trayectoria Straight Plan hacia: X={x}, Y={y}, Z={z}...'
            )

            future_plan = self.cliente_plan.call_async(req)
            plan_response = self.wait_future_in_thread(future_plan, timeout_sec=30.0)

            if plan_response is None:
                self.get_logger().error('Timeout esperando /xarm_straight_plan.')
                return

            if not plan_response.success:
                self.get_logger().error('No se pudo encontrar una trayectoria lineal válida.')
                return

            self.get_logger().info('Trayectoria encontrada. Ejecutando...')
            self.activar_controller()

            exec_req = PlanExec.Request()
            exec_req.wait = True

            future_exec = self.cliente_exec.call_async(exec_req)
            exec_response = self.wait_future_in_thread(future_exec, timeout_sec=45.0)

            if exec_response is None:
                self.get_logger().error('Timeout esperando /xarm_exec_plan.')
                return

            self.current_y = y
            self.current_z = z

            self.get_logger().info(
                f'Movimiento completado. Posición actual: '
                f'X={self.fixed_x}, Y={self.current_y}, Z={self.current_z}'
            )

        except Exception as e:
            self.get_logger().error(f'Error en secuencia de movimiento: {e}')

        finally:
            with self.lock:
                self.busy = False


def main(args=None):
    rclpy.init(args=args)
    node = LLMCartesianController()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
