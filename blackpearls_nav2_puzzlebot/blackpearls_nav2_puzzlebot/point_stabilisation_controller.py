import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool
from geometry_msgs.msg import Twist, Point
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
import transforms3d
import numpy as np

class ControllerClass(Node):
    def __init__(self):
        super().__init__('point_stabilisation_controller')
        # Parámetros de configuración
        self.declare_parameter('x', 0.0)
        self.declare_parameter('y', 0.0)
        self.declare_parameter('mode', 'bug0')  # Nuevo parámetro para seleccionar modo
        self.initial_x = self.get_parameter('x').value
        self.initial_y = self.get_parameter('y').value
        self.mode = self.get_parameter('mode').value  # Algoritmo seleccionado
        
        # Configuración de temporización
        timer_period = 0.1
        
        # Variables de estado del robot
        self.declare_parameter('goal_x', self.initial_x)
        self.declare_parameter('goal_y', self.initial_y)
        self.goal_x = self.get_parameter('goal_x').value
        self.goal_y = self.get_parameter('goal_y').value
        
        # Posición inicial del robot
        self.x = self.initial_x
        self.y = self.initial_y
        self.theta = 0.0
        
        # Parámetros del controlador
        self.declare_parameter('Kp_linear', 0.1)
        self.declare_parameter('Kp_angular', 0.3)
        self.declare_parameter('max_linear_speed', 0.3)
        self.declare_parameter('max_angular_speed', 0.3)
        self.declare_parameter('follow_distance', 0.8)
        self.declare_parameter('stop_d', 0.05)
        self.declare_parameter('goal_tolerance_distance', 0.05)
        self.declare_parameter('turning_d_deg', 5.0)

        self.Kp_linear = self.get_parameter('Kp_linear').value
        self.Kp_angular = self.get_parameter('Kp_angular').value
        self.max_linear_speed = self.get_parameter('max_linear_speed').value
        self.max_angular_speed = self.get_parameter('max_angular_speed').value
        self.follow_distance = self.get_parameter('follow_distance').value
        self.stop_d = self.get_parameter('stop_d').value
        self.goal_tolerance_distance = self.get_parameter('goal_tolerance_distance').value
        self.turning_d = np.deg2rad(self.get_parameter('turning_d_deg').value)
        
        # Flags de estado
        self.follow = False
        self.lidar_recieved = False
        self.begin_follow = False
        
        # Variables específicas para Bug2
        self.reference_line = None  # Línea de referencia (start, goal)
        self.hit_point = None       # Punto de encuentro con obstáculo
        self.hit_dist = float('inf')# Distancia al objetivo en hit_point
        self.follow_direction = 'left'  # Dirección de seguimiento (left/right)
        
        # Datos de sensores
        self.lidar = LaserScan()
        self.robot_vel = Twist()
        self.next_point = Bool()
        self.inf1 = String()
        self.inf2 = String()

        # Variables auxiliares
        self.ll = []
        self.zero = (np.zeros(360, dtype=np.float32) + 0.05).tolist()
        
        # Configuración de temporizador y suscriptores/publicadores
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.sub_p = self.create_subscription(Odometry, 'ground_truth', self.pose_callback, 10)
        self.sub_g = self.create_subscription(Point, 'goal', self.goal_callback, 10)
        self.sub_l = self.create_subscription(LaserScan, "scan", self.lidar_callback, 10)
        self.pub_v = self.create_publisher(Twist, 'cmd_vel', 10)
        self.pub = self.create_publisher(String, 'inf1', 10)
        self.pub2 = self.create_publisher(String, 'inf2', 10)

        self.start_time = self.get_clock().now()
        self.get_logger().info(f"Controlador inicializado en modo {self.mode.upper()}")

    def get_view(self, r, center, limit, max_distance):
        """Analiza una sección del escaneo LIDAR alrededor de un ángulo central"""
        self.ll = []
        detect = []
        i_i = center - limit
        i_f = center + limit
        
        # Manejo de índices circulares (0-360 grados)
        if i_i < 0:
            # Caso cuando cruza 0°
            for i in range(0, i_f):
                self.ll.append(r[i])
                detect.append(0 if r[i] > max_distance else 1)
            for i in range(360 + i_i, 360):  # Corregido: 360 + i_i
                self.ll.append(r[i])
                detect.append(0 if r[i] > max_distance else 1)
        elif i_f > 360:
            # Caso cuando cruza 360°
            for i in range(i_i, 360):
                self.ll.append(r[i])
                detect.append(0 if r[i] > max_distance else 1)
            for i in range(0, i_f - 360):
                self.ll.append(r[i])
                detect.append(0 if r[i] > max_distance else 1)
        else:
            # Caso normal
            for i in range(i_i, i_f):
                self.ll.append(r[i])
                detect.append(0 if r[i] > max_distance else 1)
        
        # Determinar si la vista está despejada
        return "clear" if np.sum(detect) == 0 else "obstacle"

    def is_on_reference_line(self, point, threshold=0.05):
        """Verifica si un punto está cerca de la línea de referencia (Bug2)"""
        if self.reference_line is None:
            return False
            
        start, goal = self.reference_line
        line_vec = np.array([goal[0] - start[0], goal[1] - start[1]])
        point_vec = np.array([point[0] - start[0], point[1] - start[1]])
        
        # Calcular proyección
        line_len = np.linalg.norm(line_vec)
        if line_len < 1e-5:
            return False
            
        # Calcular parámetro t (proyección escalar)
        t = np.dot(point_vec, line_vec) / (line_len * line_len)
        
        # Calcular punto más cercano en la línea
        closest_point = start + t * line_vec
        
        # Calcular distancia al punto más cercano
        distance = np.linalg.norm(np.array([point[0] - closest_point[0], 
                                          point[1] - closest_point[1]]))
        
        return distance < threshold and 0 <= t <= 1

    def pose_callback(self, msg):
        """Callback de odometría: actualiza posición y orientación"""
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        # Convertir quaternion a ángulos de Euler
        _, _, self.theta = transforms3d.euler.quat2euler([
            msg.pose.pose.orientation.w,
            msg.pose.pose.orientation.x,
            msg.pose.pose.orientation.y,
            msg.pose.pose.orientation.z
        ])

    def goal_callback(self, msg1):
        """Callback de objetivo: actualiza punto destino y línea de referencia"""
        self.goal_x = msg1.x
        self.goal_y = msg1.y
        
        # Para Bug2: establecer nueva línea de referencia
        self.reference_line = (np.array([self.x, self.y]), 
                               np.array([self.goal_x, self.goal_y]))
        self.hit_point = None
        self.follow = False
        self.begin_follow = False
        self.get_logger().info(f"Nuevo objetivo: ({self.goal_x}, {self.goal_y})")

    def lidar_callback(self, msg2):
        """Callback de LIDAR: almacena datos del sensor"""
        self.lidar = msg2
        self.lidar_recieved = True

    def timer_callback(self):
        """Callback periódico: lógica principal de control"""
        if not self.lidar_recieved:
            return
            
        goal_reached = False
        stopped = False
        
        # Cálculo de errores
        error_distance = np.sqrt((self.goal_x - self.x)**2 + (self.goal_y - self.y)**2)
        error_angle = np.arctan2(self.goal_y - self.y, self.goal_x - self.x) - self.theta
        error_angle = np.arctan2(np.sin(error_angle), np.cos(error_angle))  # Normalizar ángulo
        
        # Convertir ángulo a índice LIDAR (0-359)
        index_goal = int(np.rint(np.rad2deg(error_angle) + 180) % 360)
        
        # Analizar vista hacia el objetivo
        goal_view = self.get_view(self.lidar.ranges, index_goal, 12, error_distance)
        
        # Detectar obstáculo más cercano (semi-círculo frontal)
        obj_distance = min(self.lidar.ranges[:180])  # 60 grados hacia adelante (de 60° a 120°)
        a1 = 0
        a2 = 0
        
        if obj_distance < 999999:  # Valor por defecto cuando no hay obstáculo
            obstacle_index = self.lidar.ranges.index(obj_distance)
            a1 = obstacle_index - 180  # Ángulo relativo al robot
            
            # Determinar dirección de seguimiento según modo
            if self.mode == 'bug2':
                a2 = a1 + 90  # Bug2: seguir siempre a la izquierda
            else:
                a2 = a1 + 90  # Bug0: seguir a la derecha (compatibilidad con versión anterior)
            
            # Ajustes dinámicos basados en distancia
            if obj_distance < 0.2:
                a2 += 10
            if obj_distance > 0.3:
                a2 -= 5
        
        follow_angle = np.deg2rad(a2)  # Convertir a radianes

        # Lógica específica para cada algoritmo
        if self.mode == 'bug0':
            # Lógica original de Bug0
            if goal_view == "clear" or error_distance < obj_distance:
                self.follow = False
            elif goal_view == "obstacle" and obj_distance <= self.follow_distance and not self.begin_follow:
                self.follow = True
                self.begin_follow = True
                
        elif self.mode == 'bug2':
            # Lógica de Bug2
            current_pos = np.array([self.x, self.y])
            
            if goal_view == "clear" or error_distance < obj_distance:
                # Vista despejada o cerca del objetivo - desactivar seguimiento
                self.follow = False
                self.begin_follow = False
                self.hit_point = None
                self.hit_dist = float('inf')
            
            if not self.follow:
                # Modo navegación directa
                if goal_view == "obstacle" and obj_distance <= self.follow_distance:
                    # Encontrar obstáculo - activar modo seguimiento
                    self.follow = True
                    self.begin_follow = True
                    self.hit_point = current_pos
                    self.hit_dist = error_distance
                    self.get_logger().info("Obstáculo detectado. Iniciando seguimiento Bug2")
            else:
                # Modo seguimiento de obstáculo
                if self.is_on_reference_line(current_pos) and error_distance < self.hit_dist:
                    # Volver a línea en punto más cercano al objetivo
                    self.follow = False
                    self.get_logger().info("Volviendo a línea de referencia. Fin seguimiento")

        # Comportamiento de seguimiento (obstacle avoidance)
        if self.follow:
            # if np.abs(follow_angle) < self.turning_d:
                # Avanzar si el ángulo es pequeño
                self.robot_vel.linear.x = self.max_linear_speed * (obj_distance-self.stop_d)
                # self.robot_vel.angular.z = 0.0
            # else:
                # Girar para alinearse
                angular_speed = self.Kp_angular * follow_angle
                # self.robot_vel.linear.x = 0.0
                self.robot_vel.angular.z = max(min(angular_speed, self.max_angular_speed), -self.max_angular_speed)
        
        # Comportamiento normal (navegación al objetivo)
        else:
            if error_distance < self.goal_tolerance_distance:
                # Detenerse al llegar al objetivo
                self.robot_vel.linear.x = 0.0
                self.robot_vel.angular.z = 0.0
                goal_reached = True
                stopped = True
            elif np.abs(error_angle) < self.turning_d:
                # Avanzar corrigiendo dirección
                linear_speed = self.Kp_linear * error_distance
                angular_speed = self.Kp_angular * error_angle
                self.robot_vel.linear.x = max(min(linear_speed, self.max_linear_speed), -self.max_linear_speed)
                self.robot_vel.angular.z = max(min(angular_speed, self.max_angular_speed), -self.max_angular_speed)
            else:
                # Girar en el lugar para alinearse con el objetivo
                angular_speed = self.Kp_angular * error_angle
                self.robot_vel.linear.x = 0.0
                self.robot_vel.angular.z = max(min(angular_speed, self.max_angular_speed), -self.max_angular_speed)

        # Detención de emergencia por obstáculo cercano
        if obj_distance < self.stop_d:
            stopped = True
            self.robot_vel.linear.x = self.max_linear_speed * 2 * (obj_distance - self.stop_d)
            self.robot_vel.angular.z = 0.0
            self.get_logger().warn("Obstáculo muy cercano. Deteniendo robot.")

        # Publicar comando de velocidad
        self.pub_v.publish(self.robot_vel)
        
        # Preparar y publicar datos de depuración
        mode_info = f"MODE:{self.mode} FOLLOW:{self.follow}"
        obstacle_info = f"o_d:{obj_distance:.2f} o_th:{a1} f_th:{a2}"
        position_info = f"x:{self.x:.2f} y:{self.y:.2f} th:{np.rad2deg(self.theta):.2f}"
        goal_info = f"g_x:{self.goal_x:.2f} g_y:{self.goal_y:.2f}"
        error_info = f"e_d:{error_distance:.2f} e_th:{np.rad2deg(error_angle):.2f}"
        status_info = f"REACHED:{goal_reached} STOPPED:{stopped}"

        self.inf1.data = f"{mode_info} | {obstacle_info} | {position_info} | {goal_info} | {error_info} | {status_info}"
        self.inf2.data = f"LIDAR:{np.round(self.ll, 2)} | GOAL_VIEW:{goal_view} | INDEX:{index_goal}"
        
        self.pub.publish(self.inf1)
        self.pub2.publish(self.inf2)

def main(args=None):
    rclpy.init(args=args)
    controller = ControllerClass()
    try:
        rclpy.spin(controller)
    except KeyboardInterrupt:
        pass
    finally:
        controller.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()