import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import socket

class CmdVelBridge(Node):
    def __init__(self):
        super().__init__('cmd_vel_bridge_node')
        
        # --- CONFIGURACIÓN DE RED ---
        self.IP_RASPBERRY = "192.168.137.131" # Pon la IP real de tu Raspberry
        self.PUERTO_UDP = 5005
        
        self.sock_udp = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

        # Suscribirse al cerebro de Nav2
        self.subscription = self.create_subscription(
            Twist, '/cmd_vel', self.cmd_vel_callback, 10)
            
        self.get_logger().info(f"Puente UDP iniciado. Enviando comandos a {self.IP_RASPBERRY}:{self.PUERTO_UDP}")

    def cmd_vel_callback(self, msg):
        vx = msg.linear.x
        vy = -(msg.linear.y)
        omega = msg.angular.z
        
        # Empaquetamos los 3 números separados por comas
        paquete = f"{vx:.4f},{vy:.4f},{omega:.4f}"
        
        # Disparamos el paquete por Wi-Fi
        try:
            self.sock_udp.sendto(paquete.encode(), (self.IP_RASPBERRY, self.PUERTO_UDP))
        except Exception as e:
            self.get_logger().error(f"Falla de red: {e}")   

def main(args=None):
    rclpy.init(args=args)
    nodo = None
    try:
        nodo = CmdVelBridge()
        rclpy.spin(nodo)
    except KeyboardInterrupt:
        # Esto pasa si tú presionas Ctrl+C o el sistema manda a apagar
        pass
    except Exception as e:
        # Si se muere solo por un error real de código, lo atrapará aquí
        print(f"\n[!!!] ERROR FATAL EN EL NODO UDP: {e} [!!!]\n")
    finally:
        # Limpieza segura sin importar cómo murió
        if nodo is not None:
            nodo.sock_udp.close()
            nodo.destroy_node()
        if rclpy.ok():
            try:
                rclpy.shutdown()
            except Exception:
                pass

if __name__ == '__main__':
    main()


