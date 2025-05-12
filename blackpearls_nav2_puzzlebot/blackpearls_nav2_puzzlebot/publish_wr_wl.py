#! /usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry


class PublishWheelSpeeds(Node):

    def __init__(self):
        super().__init__('publish_wr_wl')

        # Suscripción al tópico de odometría de las ruedas
        self.create_subscription(Odometry, 'wheel/odometry', self.odom_cb, 10)

        # Publicador de velocidad angular de la rueda derecha (wr)
        self.pub_wr = self.create_publisher(Float32, 'VelocityEncR', 10)

        # Publicador de velocidad angular de la rueda izquierda (wl)
        self.pub_wl = self.create_publisher(Float32, 'VelocityEncL', 10)

        # Parámetros físicos del robot
        self.L = 0.52  # Separación entre ruedas [m]
        self.r = 0.14  # Radio de las ruedas [m]

        # Inicialización de mensajes a publicar
        self.wr = Float32()
        self.wl = Float32()

        # Temporizador para publicar velocidades cada dt segundos
        self.dt = 0.04  # 25 Hz
        self.timer = self.create_timer(self.dt, self.on_timer)

    def on_timer(self):
        """
        Función que publica periódicamente las velocidades de las ruedas.
        """
        self.pub_wr.publish(self.wr)
        self.pub_wl.publish(self.wl)

    def odom_cb(self, msg):
        """
        Callback que se activa al recibir datos de odometría.
        Extrae las velocidades lineal y angular del robot y calcula las velocidades de las ruedas.
        """
        v = msg.twist.twist.linear.x     # Velocidad lineal del robot
        w = msg.twist.twist.angular.z    # Velocidad angular del robot
        [self.wr.data, self.wl.data] = self.get_wheel_speeds(v, w)

    def get_wheel_speeds(self, v, w):
        """
        Calcula las velocidades angulares de las ruedas a partir de la cinemática diferencial.
        """
        wr = (2 * v + w * self.L) / (2 * self.r)
        wl = (2 * v - w * self.L) / (2 * self.r)
        return [wr, wl]


def main(args=None): 
    rclpy.init(args=args)
    f_p = PublishWheelSpeeds()

    try:
        rclpy.spin(f_p)
    except KeyboardInterrupt:
        pass
    finally:
        f_p.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__': 
    main()
