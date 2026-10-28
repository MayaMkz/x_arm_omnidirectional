import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import OccupancyGrid
from tf2_ros import TransformBroadcaster, StaticTransformBroadcaster
import cv2
import numpy as np
import math
import threading
import time
from collections import deque


class CamaraIP_UltraRapida:
    def __init__(self, url):
        self.stream = cv2.VideoCapture(url)
        self.stream.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        self.stream.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
        self.stream.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        if not self.stream.isOpened():
            raise Exception("No se pudo conectar a DroidCam.")
        self.stopped = False
        self.frame_fresco = None

    def start(self):
        threading.Thread(target=self._update, daemon=True).start()
        while self.frame_fresco is None and not self.stopped:
            time.sleep(0.05)
        print("[OK] Video de DroidCam en vivo recibido.")
        return self

    def _update(self):
        while not self.stopped:
            if self.stream.grab():
                _, img = self.stream.retrieve()
                self.frame_fresco = cv2.resize(img, (640, 480))
            else:
                self.stop()

    def read(self):
        f = self.frame_fresco
        self.frame_fresco = None
        return f

    def stop(self):
        self.stopped = True
        self.stream.release()


def euler_to_quaternion(roll, pitch, yaw):
    qx = np.sin(roll/2)*np.cos(pitch/2)*np.cos(yaw/2) - np.cos(roll/2)*np.sin(pitch/2)*np.sin(yaw/2)
    qy = np.cos(roll/2)*np.sin(pitch/2)*np.cos(yaw/2) + np.sin(roll/2)*np.cos(pitch/2)*np.sin(yaw/2)
    qz = np.cos(roll/2)*np.cos(pitch/2)*np.sin(yaw/2) - np.sin(roll/2)*np.sin(pitch/2)*np.cos(yaw/2)
    qw = np.cos(roll/2)*np.cos(pitch/2)*np.cos(yaw/2) + np.sin(roll/2)*np.sin(pitch/2)*np.sin(yaw/2)
    return [qx, qy, qz, qw]


class FiltroRobusto:
    """Filtro 3 capas para el robot (objeto móvil)."""
    def __init__(self, ventana=4, alpha_estatico=0.04,
                 alpha_movimiento=0.05, max_salto_m=0.30,
                 umbral_movimiento_m=0.20):
        self.ventana             = ventana
        self.alpha_estatico      = alpha_estatico
        self.alpha_movimiento    = alpha_movimiento
        self.max_salto_m         = max_salto_m
        self.umbral_movimiento_m = umbral_movimiento_m
        self.buf_x   = deque(maxlen=ventana)
        self.buf_y   = deque(maxlen=ventana)
        self.buf_yaw = deque(maxlen=ventana)
        self.x_filt = self.y_filt = self.yaw_filt = None

    def actualizar(self, x_crudo, y_crudo, yaw_crudo):
        if self.x_filt is None:
            self.x_filt = x_crudo; self.y_filt = y_crudo; self.yaw_filt = yaw_crudo
            self.buf_x.append(x_crudo); self.buf_y.append(y_crudo)
            self.buf_yaw.append(yaw_crudo)
            return self.x_filt, self.y_filt, self.yaw_filt

        dist = math.hypot(x_crudo - self.x_filt, y_crudo - self.y_filt)
        if dist > self.max_salto_m:
            return self.x_filt, self.y_filt, self.yaw_filt

        self.buf_x.append(x_crudo); self.buf_y.append(y_crudo)
        self.buf_yaw.append(yaw_crudo)

        x_med   = float(np.median(self.buf_x))
        y_med   = float(np.median(self.buf_y))
        sin_sum = sum(math.sin(a) for a in self.buf_yaw)
        cos_sum = sum(math.cos(a) for a in self.buf_yaw)
        yaw_med = math.atan2(sin_sum, cos_sum)

        cambio = math.hypot(x_med - self.x_filt, y_med - self.y_filt)
        alpha  = self.alpha_movimiento if cambio > self.umbral_movimiento_m else self.alpha_estatico

        self.x_filt   = alpha*x_med   + (1-alpha)*self.x_filt
        self.y_filt   = alpha*y_med   + (1-alpha)*self.y_filt
        diff_yaw      = math.atan2(math.sin(yaw_med-self.yaw_filt),
                                   math.cos(yaw_med-self.yaw_filt))
        self.yaw_filt = self.yaw_filt + alpha*diff_yaw
        return self.x_filt, self.y_filt, self.yaw_filt


class FiltroEstacion:
    """
    Filtro especial para ArUcos de estaciones FIJAS.
    Estrategia completamente diferente al robot:
      - Las estaciones NO se mueven → acumulamos muchas mediciones
        y tomamos la mediana global. Cuantas más mediciones, más estable.
      - El yaw lo fijamos manualmente porque a 3m de altura con markers
        pequeños solvePnP no puede calcularlo con precisión suficiente.
      - Una vez que tenemos suficientes muestras (MUESTRAS_MINIMAS),
        la posición se "congela" y deja de actualizarse con ruido.
    """
    def __init__(self, yaw_fijo: float, muestras_minimas: int = 30,
                 max_muestras: int = 100, max_salto_m: float = 0.08):
        """
        yaw_fijo:        orientación fija de la estación (no se calcula desde visión)
        muestras_minimas: cuántas mediciones acumular antes de publicar TF
        max_muestras:    tamaño máximo del buffer (mediana sobre estas muestras)
        max_salto_m:     rechaza mediciones que salten más de esto
        """
        self.yaw_fijo        = yaw_fijo
        self.muestras_minimas = muestras_minimas
        self.max_salto_m     = max_salto_m
        self.buf_x = deque(maxlen=max_muestras)
        self.buf_y = deque(maxlen=max_muestras)
        self.x_est = None   # estimación actual
        self.y_est = None
        self.listo = False   # True cuando tenemos suficientes muestras

    def agregar(self, x_crudo, y_crudo):
        """
        Agrega una nueva medición. Retorna (x, y, yaw, listo)
        donde listo=True cuando ya hay suficientes muestras para publicar.
        """
        # Rechazar outliers respecto a la mediana actual si ya tenemos datos
        if len(self.buf_x) >= 5:
            x_med_actual = float(np.median(self.buf_x))
            y_med_actual = float(np.median(self.buf_y))
            dist = math.hypot(x_crudo - x_med_actual, y_crudo - y_med_actual)
            if dist > self.max_salto_m:
                # Outlier — ignorar esta medición
                if self.listo:
                    return self.x_est, self.y_est, self.yaw_fijo, True
                return None, None, None, False

        self.buf_x.append(x_crudo)
        self.buf_y.append(y_crudo)

        if len(self.buf_x) >= self.muestras_minimas:
            self.x_est = float(np.median(self.buf_x))
            self.y_est = float(np.median(self.buf_y))
            self.listo = True

        if self.listo:
            return self.x_est, self.y_est, self.yaw_fijo, True
        return None, None, None, False

    @property
    def n_muestras(self):
        return len(self.buf_x)


class VisionGPSNode(Node):
    def __init__(self):
        super().__init__('vision_gps_node')

        self.MAPA_RESOLUCION = 0.01
        self.MAPA_ANCHO_PX   = 300
        self.MAPA_ALTO_PX    = 380

        self.MARKER_SIZE     = 0.136
        self.ID_ORIGEN       = 1
        self.OFFSET_ORIGEN_X = 1.50
        self.OFFSET_ORIGEN_Y = 1.90

        self.ROBOT_TAGS = {
            5: (-0.230,  0.120),
            4: ( 0.230,  0.120),
            3: (-0.230, -0.120),
            0: ( 0.230, -0.120),
        }
        self.PRIORIDAD_TAGS = [0, 3, 4, 5]
        self.TAG_TIMEOUT_S  = 1.25

        self.TAGS_ESTACION = {
            6: 'pick',
            7: 'classif',
            2: 'home',
        }
        self.ESTACIONES_CON_CAJA = {'pick', 'classif'}
        self.CAJA_LADO_M   = 0.38
        self.CAJA_OFFSET_M = 0.19
        self.MURO_GROSOR_M = 0.05

        self.memoria_tags         = {}
        self.tag_robot_activo     = None
        self.tvec_origen_guardado = None
        self.rvec_origen_guardado = None

        # Filtro robusto para el robot (móvil)
        self.filtro_robot = FiltroRobusto(
            ventana=5,
            alpha_movimiento=0.40,
            max_salto_m=0.50,
            umbral_movimiento_m=0.10,
        )

        # ── CLAVE: filtros de estación fija ──────────────────────────
        # yaw_fijo: la orientación real de cada estación en tu mapa.
        # Ajusta estos valores según cómo están orientados físicamente.
        # Si no sabes el yaw exacto, pon 0.0 — lo que importa es x,y.
        self.filtros_estacion = {
            'pick':    FiltroEstacion(yaw_fijo=1.57, muestras_minimas=30, max_muestras=200),
            'classif': FiltroEstacion(yaw_fijo=1.57, muestras_minimas=30, max_muestras=200),
            'home':    FiltroEstacion(yaw_fijo=1.57, muestras_minimas=30, max_muestras=200),
        }

        # Filtro de mediana para el origen (ArUco ID 1)
        self.filtro_origen_x  = deque(maxlen=20)
        self.filtro_origen_y  = deque(maxlen=20)
        self.filtro_origen_z  = deque(maxlen=20)
        self.tvec_origen_filt = None

        self.estaciones_pose = {}   # nombre → (x, y, yaw) — solo cuando listo
        self.cajas_pose      = {}

        self.DROIDCAM_URL = "http://192.168.137.246:4747/video"

        self.tf_broadcaster        = TransformBroadcaster(self)
        self.tf_static_broadcaster = StaticTransformBroadcaster(self)
        self.pub_cajas = self.create_publisher(OccupancyGrid, '/cajas_obstaculos', 10)

        self.aruco_dict   = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
        self.aruco_params = cv2.aruco.DetectorParameters()
        self.aruco_params.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
        self.aruco_params.adaptiveThreshWinSizeStep = 10
        self.detector = cv2.aruco.ArucoDetector(self.aruco_dict, self.aruco_params)

        hs = self.MARKER_SIZE / 2.0
        self.obj_points = np.array([
            [-hs,  hs, 0], [hs,  hs, 0], [hs, -hs, 0], [-hs, -hs, 0]
        ], dtype=np.float32)

        self.get_logger().info(f"Conectando a DroidCam: {self.DROIDCAM_URL}")
        try:
            self.cap = CamaraIP_UltraRapida(self.DROIDCAM_URL).start()
        except Exception as e:
            self.get_logger().error(str(e))
            raise SystemExit

        fs = cv2.FileStorage(
            "/home/jerry/x_arm/src/xarm_ros2/logica_almacen/logica_almacen/vision/parametros_droidcam.yaml",
            cv2.FILE_STORAGE_READ)
        if fs.isOpened():
            matK = fs.getNode("camera_matrix").mat()
            matD = fs.getNode("dist_coeffs").mat()
            if matK is not None and matD is not None:
                self.K = np.array(matK, dtype=np.float32)
                self.D = np.array(matD, dtype=np.float32)
                self.get_logger().info("[OK] Calibración cargada.")
            fs.release()
        else:
            f_est = 640 * 0.9
            self.K = np.array([[f_est,0,320],[0,f_est,240],[0,0,1]], dtype=np.float32)
            self.D = np.zeros((4,1), dtype=np.float32)

        self.timer       = self.create_timer(0.05, self.procesar_frame)
        self.timer_cajas = self.create_timer(1.0,  self.publicar_mapa_cajas)

    # ------------------------------------------------------------------
    def _actualizar_origen_filtrado(self, tvec_nuevo):
        self.filtro_origen_x.append(float(tvec_nuevo[0]))
        self.filtro_origen_y.append(float(tvec_nuevo[1]))
        self.filtro_origen_z.append(float(tvec_nuevo[2]))
        self.tvec_origen_filt = np.array([
            [np.median(self.filtro_origen_x)],
            [np.median(self.filtro_origen_y)],
            [np.median(self.filtro_origen_z)],
        ], dtype=np.float64)
        return self.tvec_origen_filt

    def tvec_a_mapa_xy(self, tvec, R_origen, tvec_origen):
        """Solo calcula x,y — NO el yaw (lo fija FiltroEstacion)."""
        pos_mapa = R_origen.T @ (tvec - tvec_origen)
        x = float(pos_mapa[0][0]) + self.OFFSET_ORIGEN_X
        y = float(pos_mapa[1][0]) + self.OFFSET_ORIGEN_Y
        return x, y

    def tvec_a_mapa(self, tvec, rvec, R_origen, tvec_origen):
        R_tag, _ = cv2.Rodrigues(rvec)
        pos_mapa  = R_origen.T @ (tvec - tvec_origen)
        x = float(pos_mapa[0][0]) + self.OFFSET_ORIGEN_X
        y = float(pos_mapa[1][0]) + self.OFFSET_ORIGEN_Y
        R_tag_mapa = R_origen.T @ R_tag
        yaw = math.atan2(math.sin(math.atan2(-R_tag_mapa[1,0], -R_tag_mapa[0,0]) + math.pi/2),
                         math.cos(math.atan2(-R_tag_mapa[1,0], -R_tag_mapa[0,0]) + math.pi/2))
        return x, y, yaw

    def calcular_pos_caja(self, tvec, rvec, R_origen, tvec_origen):
        R_tag, _ = cv2.Rodrigues(rvec)
        frente   = np.array([[0.0], [-self.CAJA_OFFSET_M], [0.0]], dtype=np.float64)
        caja_cam = tvec + R_tag @ frente
        pos_mapa = R_origen.T @ (caja_cam - tvec_origen)
        cx = float(pos_mapa[0][0]) + self.OFFSET_ORIGEN_X
        cy = float(pos_mapa[1][0]) + self.OFFSET_ORIGEN_Y
        R_tag_mapa = R_origen.T @ R_tag
        yaw = math.atan2(math.sin(math.atan2(-R_tag_mapa[1,0], -R_tag_mapa[0,0]) + math.pi/2),
                         math.cos(math.atan2(-R_tag_mapa[1,0], -R_tag_mapa[0,0]) + math.pi/2))
        return cx, cy, yaw

    def publicar_tf_estacion(self, nombre, x, y, yaw):
        t = TransformStamped()
        t.header.stamp    = self.get_clock().now().to_msg()
        t.header.frame_id = 'map'
        t.child_frame_id  = f'estacion_{nombre}'
        t.transform.translation.x = x
        t.transform.translation.y = y
        t.transform.translation.z = 0.0
        q = euler_to_quaternion(0, 0, yaw)
        t.transform.rotation.x = q[0]; t.transform.rotation.y = q[1]
        t.transform.rotation.z = q[2]; t.transform.rotation.w = q[3]
        self.tf_static_broadcaster.sendTransform(t)

    # ------------------------------------------------------------------
    def publicar_mapa_cajas(self):
        if not self.cajas_pose:
            return
        grid = OccupancyGrid()
        grid.header.stamp    = self.get_clock().now().to_msg()
        grid.header.frame_id = 'map'
        grid.info.resolution = self.MAPA_RESOLUCION
        grid.info.width      = self.MAPA_ANCHO_PX
        grid.info.height     = self.MAPA_ALTO_PX
        grid.info.origin.orientation.w = 1.0
        datos = [-1] * (self.MAPA_ANCHO_PX * self.MAPA_ALTO_PX)
        for nombre, (cx, cy, yaw) in self.cajas_pose.items():
            self._dibujar_caja_3_lados(datos, cx, cy, yaw)
        grid.data = datos
        self.pub_cajas.publish(grid)

    def _marcar_celda(self, datos, xm, ym):
        px = int(xm / self.MAPA_RESOLUCION)
        py = int(ym / self.MAPA_RESOLUCION)
        if 0 <= px < self.MAPA_ANCHO_PX and 0 <= py < self.MAPA_ALTO_PX:
            datos[py * self.MAPA_ANCHO_PX + px] = 100

    def _dibujar_segmento(self, datos, x0, y0, x1, y1, grosor_m):
        longitud = math.hypot(x1-x0, y1-y0)
        if longitud < 1e-6: return
        dx = (x1-x0)/longitud; dy = (y1-y0)/longitud
        nx = -dy; ny = dx
        paso = self.MAPA_RESOLUCION; mitad = grosor_m/2.0
        for i in range(int(longitud/paso)+1):
            t=i*paso; xc=x0+dx*t; yc=y0+dy*t
            for j in range(-int(mitad/paso)-1, int(mitad/paso)+2):
                w=j*paso
                if abs(w)<=mitad: self._marcar_celda(datos, xc+nx*w, yc+ny*w)

    def _dibujar_caja_3_lados(self, datos, cx, cy, yaw):
        L=self.CAJA_LADO_M/2.0; g=self.MURO_GROSOR_M
        cos_y=math.cos(yaw); sin_y=math.sin(yaw)
        def lm(xl,yl): return (cx+xl*cos_y-yl*sin_y, cy+xl*sin_y+yl*cos_y)
        fi=lm(-L,L); fd=lm(L,L); ti=lm(-L,-L); td=lm(L,-L)
        self._dibujar_segmento(datos,ti[0],ti[1],td[0],td[1],g)
        self._dibujar_segmento(datos,fi[0],fi[1],ti[0],ti[1],g)
        self._dibujar_segmento(datos,fd[0],fd[1],td[0],td[1],g)

    # ------------------------------------------------------------------
    def procesar_frame(self):
        frame = self.cap.read()
        if frame is None: return

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        corners, ids, _ = self.detector.detectMarkers(gray)
        ahora = time.time()

        overlay = frame.copy()
        cv2.rectangle(overlay,(10,10),(420,170),(15,15,15),-1)
        cv2.addWeighted(overlay,0.75,frame,0.25,0,frame)
        cv2.putText(frame,"PICKASSO AMR - VISION GLOBAL",(20,35),
                    cv2.FONT_HERSHEY_SIMPLEX,0.6,(255,255,255),2)
        cv2.line(frame,(20,45),(400,45),(100,100,100),1)

        if ids is not None:
            cv2.aruco.drawDetectedMarkers(frame, corners)
            for i, mid_arr in enumerate(ids):
                mid = int(mid_arr[0])
                ok, rvec, tvec = cv2.solvePnP(
                    self.obj_points, corners[i][0], self.K, self.D,
                    flags=cv2.SOLVEPNP_IPPE_SQUARE)
                if not ok: continue

                self.memoria_tags[mid] = {'rvec': rvec, 'tvec': tvec, 'ts': ahora}

                if mid == self.ID_ORIGEN:
                    self.tvec_origen_guardado = self._actualizar_origen_filtrado(tvec)
                    self.rvec_origen_guardado = rvec

                cv2.drawFrameAxes(frame, self.K, self.D, rvec, tvec, 0.04)
                R_tag, _ = cv2.Rodrigues(rvec)
                frente_cam = R_tag @ np.array([[0.0],[-0.08],[0.0]]) + tvec
                p0 = cv2.projectPoints(tvec,np.zeros((3,1)),np.zeros((3,1)),self.K,self.D)[0][0][0]
                p1 = cv2.projectPoints(frente_cam,np.zeros((3,1)),np.zeros((3,1)),self.K,self.D)[0][0][0]
                cv2.arrowedLine(frame,(int(p0[0]),int(p0[1])),(int(p1[0]),int(p1[1])),(0,255,255),3,tipLength=0.3)

                # ── ESTACIONES: solo x,y desde visión, yaw fijo ──────
                if mid in self.TAGS_ESTACION and self.tvec_origen_guardado is not None:
                    R_origen, _ = cv2.Rodrigues(self.rvec_origen_guardado)
                    nombre = self.TAGS_ESTACION[mid]

                    # Solo calculamos x,y — NO el yaw
                    x_crudo, y_crudo = self.tvec_a_mapa_xy(
                        tvec, R_origen, self.tvec_origen_guardado)

                    filtro = self.filtros_estacion[nombre]
                    x_est, y_est, yaw_est, listo = filtro.agregar(x_crudo, y_crudo)

                    if listo:
                        self.estaciones_pose[nombre] = (x_est, y_est, yaw_est)
                        self.publicar_tf_estacion(nombre, x_est, y_est, yaw_est)

                        if nombre in self.ESTACIONES_CON_CAJA:
                            # Para la caja usamos la posición filtrada + yaw fijo
                            yaw_fijo = filtro.yaw_fijo
                            cos_y = math.cos(yaw_fijo); sin_y = math.sin(yaw_fijo)
                            # Centro de la caja: CAJA_OFFSET_M en dirección -Y del tag
                            cx = x_est - math.sin(yaw_fijo) * self.CAJA_OFFSET_M
                            cy = y_est + math.cos(yaw_fijo) * self.CAJA_OFFSET_M
                            self.cajas_pose[nombre] = (cx, cy, yaw_fijo)

        # Tag activo del robot
        nuevo_activo = None
        if (self.tag_robot_activo is not None
                and self.tag_robot_activo in self.memoria_tags
                and (ahora - self.memoria_tags[self.tag_robot_activo]['ts']) < self.TAG_TIMEOUT_S):
            nuevo_activo = self.tag_robot_activo
        else:
            for tid in self.PRIORIDAD_TAGS:
                if tid in self.memoria_tags and (ahora-self.memoria_tags[tid]['ts']) < self.TAG_TIMEOUT_S:
                    nuevo_activo = tid; break
        self.tag_robot_activo = nuevo_activo

        # HUD
        estado = "FIJADO" if self.tvec_origen_guardado is not None else "BUSCANDO..."
        color  = (0,255,0) if self.tvec_origen_guardado is not None else (0,0,255)
        cv2.putText(frame,f"Origen: {estado}",(20,65),cv2.FONT_HERSHEY_SIMPLEX,0.5,color,1)

        y_hud = 83
        for nombre, filt in self.filtros_estacion.items():
            n = filt.n_muestras
            if nombre in self.estaciones_pose:
                x,y,_ = self.estaciones_pose[nombre]
                txt = f"{nombre.upper()}: ({x:.3f},{y:.3f}) [{n} muestras OK]"
                col = (180,255,180)
            else:
                txt = f"{nombre.upper()}: calibrando... {n}/30"
                col = (100,200,255)
            cv2.putText(frame, txt, (20,y_hud), cv2.FONT_HERSHEY_SIMPLEX, 0.42, col, 1)
            y_hud += 17

        # TF del robot
        if self.tag_robot_activo is not None and self.tvec_origen_guardado is not None:
            tag = self.memoria_tags[self.tag_robot_activo]
            off_x, off_y = self.ROBOT_TAGS[self.tag_robot_activo]
            R_tag, _ = cv2.Rodrigues(tag['rvec'])
            offset_cam  = R_tag @ np.array([[off_x],[off_y],[0.0]], dtype=np.float64)
            T_robot_cam = tag['tvec'] + offset_cam
            R_origen, _ = cv2.Rodrigues(self.rvec_origen_guardado)
            pos_mapa    = R_origen.T @ (T_robot_cam - self.tvec_origen_guardado)
            cx_mapa = float(pos_mapa[0][0]) + self.OFFSET_ORIGEN_X
            cy_mapa = float(pos_mapa[1][0]) + self.OFFSET_ORIGEN_Y
            R_robot_mapa = R_origen.T @ R_tag
            yaw_base  = math.atan2(-R_robot_mapa[1,0], -R_robot_mapa[0,0])
            yaw_crudo = math.atan2(math.sin(yaw_base+math.pi/2),math.cos(yaw_base+math.pi/2))

            x_filt, y_filt, yaw_filt = self.filtro_robot.actualizar(cx_mapa, cy_mapa, yaw_crudo)

            t = TransformStamped()
            t.header.stamp    = self.get_clock().now().to_msg()
            t.header.frame_id = 'odom'
            t.child_frame_id  = 'base_link'
            t.transform.translation.x = x_filt
            t.transform.translation.y = y_filt
            t.transform.translation.z = 0.0
            q = euler_to_quaternion(0, 0, yaw_filt)
            t.transform.rotation.x=q[0]; t.transform.rotation.y=q[1]
            t.transform.rotation.z=q[2]; t.transform.rotation.w=q[3]
            self.tf_broadcaster.sendTransform(t)

            cv2.putText(frame,f"Robot[{self.tag_robot_activo}]: X:{x_filt:+.3f} Y:{y_filt:+.3f}",
                        (20,y_hud+5),cv2.FONT_HERSHEY_SIMPLEX,0.55,(0,255,0),2)
        elif self.tvec_origen_guardado is not None:
            cv2.putText(frame,"Esperando Pickasso...",(20,y_hud+5),
                        cv2.FONT_HERSHEY_SIMPLEX,0.5,(0,255,255),1)

        cv2.imshow("Camara - Nav2", frame)
        cv2.waitKey(1)


def main(args=None):
    rclpy.init(args=args)
    try:
        nodo = VisionGPSNode()
        rclpy.spin(nodo)
    except SystemExit: pass
    except KeyboardInterrupt: pass
    finally:
        if 'nodo' in locals():
            nodo.cap.stop()
            nodo.destroy_node()
        cv2.destroyAllWindows()
        rclpy.shutdown()

if __name__ == '__main__':
    main()