#!/usr/bin/env python3
"""
holonomic_controller_v4.py — Pickasso AMR
==========================================
Fix vs v3:
  - SÍ acepta replanning continuo (retroalimentación de visión activa)
  - Pero filtra planes con goal casi idéntico al actual (tolerancia 5cm)
    → evita el salto de targets por el snapping del grid NavFn
  - El lookahead siempre busca hacia adelante en el plan desde
    la posición actual del robot → sigue la ruta actualizada
  - Si el goal cambia más de 10cm → acepta el plan nuevo (obstáculo
    detectado o corrección de localización)
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Path
import math
import tf2_ros
from tf2_ros import TransformException
import time

class HolonomicController(Node):
    def __init__(self):
        super().__init__('holonomic_controller')

        # --- Parámetros de control ---
        self.lookahead_dist      = 0.4    # m
        self.kp_xy               = 0.9
        self.kp_w                = 1.0
        self.max_vx              = 0.30   # m/s
        self.max_vy              = 0.30   # m/s
        self.max_w               = 0.50   # rad/s
        self.goal_tolerance      = 0.08   # m — tolerancia de llegada
        self.yaw_tolerance       = 0.10   # rad
        # Si el goal del nuevo plan difiere más de esto → actualizar plan
        # Si difiere menos → es solo ruido del grid, ignorar
        self.goal_update_thresh  = 0.10   # m  (10 cm)

        # Estado
        self.plan           = []
        self.goal_pose      = None
        self.active         = False
        self.goal_reached_time = None
        self.cooldown_sec   = 1.0
        self.plan_count     = 0   # debug: cuántos planes aceptamos

        # TF
        self.tf_buffer   = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.sub_plan = self.create_subscription(
            Path, '/received_global_plan', self.cb_plan, 10)
        self.pub_cmd  = self.create_publisher(Twist, '/cmd_vel', 10)
        self.timer    = self.create_timer(0.1, self.control_loop)

        self.get_logger().info("HolonomicController v4 listo.")
        self.get_logger().info(
            f"goal_update_thresh={self.goal_update_thresh}m — "
            "acepta replanning si el goal cambia >10cm")

    def cb_plan(self, msg: Path):
        if len(msg.poses) == 0:
            return

        # Cooldown tras llegar al goal anterior
        if self.goal_reached_time is not None:
            if time.time() - self.goal_reached_time < self.cooldown_sec:
                return

        new_goal = msg.poses[-1].pose

        # --- FILTRO DE REPLANNING ---
        # Si ya tenemos un plan activo, solo lo reemplazamos si el goal
        # nuevo está suficientemente lejos del goal actual.
        # Esto filtra el ruido de snapping del grid (~1-3cm de variación)
        # pero deja pasar cambios reales (obstáculo, corrección de pose).
        if self.active and self.goal_pose is not None:
            dist_goal_change = math.hypot(
                new_goal.position.x - self.goal_pose.position.x,
                new_goal.position.y - self.goal_pose.position.y)

            if dist_goal_change < self.goal_update_thresh:
                # Ruido de grid — actualizar la ruta pero mantener el goal fijo
                # Esto sí nos da retroalimentación de obstáculos y localización
                self.plan = msg.poses
                return  # sin log para no saturar
            else:
                # Goal cambió significativamente → plan nuevo real
                self.get_logger().info(
                    f"[REPLANNING] Goal cambió {dist_goal_change:.3f}m → "
                    f"nuevo goal: ({new_goal.position.x:.3f}, {new_goal.position.y:.3f})")

        # Aceptar plan y goal nuevos
        self.plan      = msg.poses
        self.goal_pose = new_goal
        self.active    = True
        self.goal_reached_time = None
        self.plan_count += 1

        self.get_logger().info(
            f"[PLAN #{self.plan_count}] {len(self.plan)} puntos | "
            f"Goal: x={self.goal_pose.position.x:.3f} "
            f"y={self.goal_pose.position.y:.3f}")

    def get_robot_pose(self):
        try:
            t = self.tf_buffer.lookup_transform(
                'map', 'base_link',
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=0.1))
            x   = t.transform.translation.x
            y   = t.transform.translation.y
            q   = t.transform.rotation
            yaw = math.atan2(
                2.0*(q.w*q.z + q.x*q.y),
                1.0 - 2.0*(q.y*q.y + q.z*q.z))
            return x, y, yaw
        except TransformException as e:
            self.get_logger().warn(f"TF no disponible: {e}",
                                   throttle_duration_sec=2.0)
            return None

    def get_yaw(self, orientation):
        q = orientation
        return math.atan2(
            2.0*(q.w*q.z + q.x*q.y),
            1.0 - 2.0*(q.y*q.y + q.z*q.z))

    def find_lookahead(self, rx, ry):
        """
        Busca el lookahead point en el plan actual.
        Primero encuentra el punto más cercano al robot,
        luego busca hacia adelante en la ruta desde ahí.
        Esto evita que el robot persiga un punto detrás de él
        cuando el plan se actualiza.
        """
        if not self.plan:
            return None, None

        # 1. Encontrar el índice del punto más cercano al robot
        min_dist  = float('inf')
        min_idx   = 0
        for i, ps in enumerate(self.plan):
            d = math.hypot(ps.pose.position.x - rx,
                           ps.pose.position.y - ry)
            if d < min_dist:
                min_dist = d
                min_idx  = i

        # 2. Desde ese punto, buscar hacia adelante a lookahead_dist
        for ps in self.plan[min_idx:]:
            px = ps.pose.position.x
            py = ps.pose.position.y
            if math.hypot(px - rx, py - ry) >= self.lookahead_dist:
                return px, py

        # Si no hay punto a esa distancia → usar el goal directamente
        last = self.plan[-1].pose.position
        return last.x, last.y

    def world_to_robot(self, wx, wy, rx, ry, ryaw):
        dx = wx - rx
        dy = wy - ry
        local_x =  dx * math.cos(ryaw) + dy * math.sin(ryaw)
        local_y = -dx * math.sin(ryaw) + dy * math.cos(ryaw)
        return local_x, local_y

    def clamp(self, v, limit):
        return max(-limit, min(limit, v))

    def stop(self):
        self.pub_cmd.publish(Twist())

    def control_loop(self):
        if not self.active:
            return

        result = self.get_robot_pose()
        if result is None:
            self.stop()
            return

        rx, ry, ryaw = result
        gx = self.goal_pose.position.x
        gy = self.goal_pose.position.y
        dist_goal = math.hypot(gx - rx, gy - ry)

        # ¿Llegamos?
        if dist_goal < self.goal_tolerance:
            goal_yaw  = self.get_yaw(self.goal_pose.orientation)
            yaw_error = math.atan2(
                math.sin(goal_yaw - ryaw),
                math.cos(goal_yaw - ryaw))

            if abs(yaw_error) > self.yaw_tolerance:
                cmd = Twist()
                cmd.angular.z = self.clamp(self.kp_w * yaw_error, self.max_w)
                self.pub_cmd.publish(cmd)
            else:
                self.stop()
                self.active = False
                self.goal_reached_time = time.time()
                self.get_logger().info(
                    f"¡GOAL ALCANZADO! dist_final={dist_goal:.3f}m")
            return

        # Lookahead → frame del robot
        lx, ly = self.find_lookahead(rx, ry)
        if lx is None:
            self.stop()
            return

        err_x, err_y = self.world_to_robot(lx, ly, rx, ry, ryaw)

        vx = self.clamp(self.kp_xy * err_x, self.max_vx)
        vy = self.clamp(self.kp_xy * err_y, self.max_vy)

        # Corrección de yaw continua
        goal_yaw  = self.get_yaw(self.goal_pose.orientation)
        yaw_error = math.atan2(
            math.sin(goal_yaw - ryaw),
            math.cos(goal_yaw - ryaw))
        w = self.clamp(self.kp_w * yaw_error, self.max_w)

        cmd = Twist()
        cmd.linear.x  = vx
        cmd.linear.y  = vy
        cmd.angular.z = w
        self.pub_cmd.publish(cmd)

        self.get_logger().info(
            f"pos:({rx:.2f},{ry:.2f}) yaw:{math.degrees(ryaw):.0f}° "
            f"dist_goal:{dist_goal:.2f}m | "
            f"look:({lx:.2f},{ly:.2f}) "
            f"err_robot:({err_x:+.2f},{err_y:+.2f}) | "
            f"vx:{vx:+.3f} vy:{vy:+.3f} w:{w:+.3f}",
            throttle_duration_sec=0.3)


def main(args=None):
    rclpy.init(args=args)
    node = HolonomicController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()