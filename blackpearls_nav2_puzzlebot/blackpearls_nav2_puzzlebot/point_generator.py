import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, String
from geometry_msgs.msg import PoseStamped
import numpy as np

class PointGeneratorClass(Node):
    def __init__(self):
        super().__init__('point_generator')

        # Parámetro configurable para elegir la figura
        self.declare_parameter('figure', 0)

        # Timer para ejecutar el callback cada 0.1 segundos
        timer_period = 0.1

        # Estados de control para el avance entre puntos
        self.next = False
        self.prev = False

        # Rango de figuras (de 3 hasta 7 lados)
        self.n = 3
        self.end = 7
        self.r = 0.5  # Radio de las figuras

        # Índices y almacenamiento
        self.index = 0
        self.figure = 0
        self.figure_prev = 0
        self.points = self.points_make()  # Generar puntos
        self.goal = PoseStamped()         # Mensaje del punto objetivo
        self.inf1 = String()              # Mensaje informativo

        # Timer principal
        self.timer = self.create_timer(timer_period, self.timer_callback)

        # Subscripción para recibir señal de avanzar al siguiente punto
        self.sub_n = self.create_subscription(Bool, 'next_point', self.next_callback, 10)

        # Publicadores: objetivo actual y figura actual
        self.pub_g = self.create_publisher(PoseStamped, 'set_point', 10)
        self.pub = self.create_publisher(String, 'inf', 10)

        # Guardar el tiempo de inicio (no se usa activamente)
        self.start_time = self.get_clock().now()

    def points_make(self):
        """
        Genera una lista de listas con los puntos correspondientes a figuras
        con lados de n a end. Cada figura contiene puntos distribuidos circularmente.
        """
        n = self.n
        points = []

        while n <= self.end:
            fraction = 2.0 * np.pi / float(n)
            l = []

            for i in range(n):
                x = float(((np.cos((i+1) * fraction) * self.r) - self.r) * -1)
                y = float(np.sin((i+1) * fraction) * self.r)
                l.append([x, y])

            # Rellenar con ceros para mantener longitud constante
            for i in range(self.end - n):
                l.append([0.0, 0.0])

            points.append(l)
            n += 1

        return points

    def timer_callback(self):
        """
        Se ejecuta periódicamente para actualizar y publicar el punto objetivo
        dependiendo del índice actual y la figura seleccionada.
        """
        self.figure = self.get_parameter('figure').get_parameter_value().integer_value

        # Reiniciar índice si se cambia de figura
        if self.figure != self.figure_prev:
            self.index = 0

        # Solo avanzar si se activó el 'next' y no se repite el estado anterior
        if self.next and not self.prev:
            self.index += 1

        # Publicar el siguiente punto si la figura está dentro del rango válido
        if self.n <= self.figure <= self.end:
            self.goal.pose.position.x = self.points[self.figure - 3][self.index][0]
            self.goal.pose.position.y = self.points[self.figure - 3][self.index][1]
            self.pub_g.publish(self.goal)

        # Publicar mensaje informativo con el número de figura actual
        self.inf1.data = str(self.figure)
        self.pub.publish(self.inf1)

        # Actualizar estados previos
        self.prev = self.next
        self.figure_prev = self.figure

    def next_callback(self, msg):
        """
        Callback que recibe un mensaje Bool para avanzar al siguiente punto.
        """
        self.next = bool(msg.data)

def main(args=None):
    rclpy.init(args=args)
    node3 = PointGeneratorClass()
    try:
        rclpy.spin(node3)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            rclpy.shutdown()
        node3.destroy_node()

if __name__ == '__main__':
    main()
