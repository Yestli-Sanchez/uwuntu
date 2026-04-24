import rclpy
from rclpy.node import Node
from tf2_ros import TransformBroadcaster, StaticTransformBroadcaster
from geometry_msgs.msg import TransformStamped
import transforms3d
import numpy as np

class PuzzlebotPublisher(Node):
    def __init__(self):
        super().__init__('puzzlebot_publisher')
        
        # Broadcasters Estáticos
        #self.map_broadcaster = StaticTransformBroadcaster(self)
        self.base_link_broadcaster = StaticTransformBroadcaster(self)
        self.caster_broadcaster = StaticTransformBroadcaster(self)

        # Broadcasters Dinámicos
        self.odom_broadcaster = TransformBroadcaster(self)

        # Configuración TF: world -> map (Estático)
        #t_map = TransformStamped()
        #t_map.header.stamp = self.get_clock().now().to_msg()
        #t_map.child_frame_id = 'map'
        #t_map.transform.translation.y = 0.0
        #t_map.transform.translation.z = 0.0
        #q = transforms3d.euler.euler2quat(0, 0, 0)
        #t_map.transform.rotation.x = q[1]
        #t_map.transform.rotation.y = q[2]
        #t_map.transform.rotation.z = q[3]
        #t_map.transform.rotation.w = q[0]

        # 2. Configuración TF: base_footprint -> base_link (Estático)
        # Eleva el cuerpo del robot 5cm sobre el nivel del suelo
        t_base_link = TransformStamped()
        t_base_link.header.stamp = self.get_clock().now().to_msg()
        t_base_link.header.frame_id = 'base_footprint'
        t_base_link.child_frame_id = 'base_link'
        t_base_link.transform.translation.x = 0.0
        t_base_link.transform.translation.y = 0.0
        t_base_link.transform.translation.z = 0.05
        q = transforms3d.euler.euler2quat(0, 0, 0)
        t_base_link.transform.rotation.x = q[1]
        t_base_link.transform.rotation.y = q[2]
        t_base_link.transform.rotation.z = q[3]
        t_base_link.transform.rotation.w = q[0]

        # 3. Configuración TF: base_link -> caster (Estático)
        # Posiciona la rueda loca en la parte trasera del robot
        t_caster = TransformStamped()
        t_caster.header.stamp = self.get_clock().now().to_msg()
        t_caster.header.frame_id = 'base_link'
        t_caster.child_frame_id = 'caster'
        t_caster.transform.translation.x = -0.09
        t_caster.transform.translation.y = 0.0
        t_caster.transform.translation.z = -0.043
        q = transforms3d.euler.euler2quat(0, 0, 0)
        t_caster.transform.rotation.x = q[1]
        t_caster.transform.rotation.y = q[2]
        t_caster.transform.rotation.z = q[3]
        t_caster.transform.rotation.w = q[0]

        # Enviamos las transformaciones estáticas una sola vez
        #self.map_broadcaster.sendTransform([t_map])
        self.base_link_broadcaster.sendTransform([t_base_link])
        self.caster_broadcaster.sendTransform([t_caster])

        # Inicialización de mensajes para las transformaciones dinámicas (que cambian en el timer)
        self.odom_transform = TransformStamped()
        self.base_footprint_transform = TransformStamped()
        self.wheel_r_transform = TransformStamped()
        self.wheel_l_transform = TransformStamped()
        
        # Broadcasters para los componentes móviles
        self.tf_odom = TransformBroadcaster(self)
        self.tf_base_footprint = TransformBroadcaster(self)
        self.tf_wheel_r = TransformBroadcaster(self)
        self.tf_wheel_l = TransformBroadcaster(self)

        # Configuración del Timer: Ejecuta la función timer_cb 20 veces por segundo
        timer_period = 0.05  # 20 Hz
        self.timer = self.create_timer(timer_period, self.timer_cb)

        # Parámetros del movimiento (Trayectoria circular)
        self.start_time = self.get_clock().now()
        self.radius = 0.7  # radio del movimiento circular
        self.angular_speed = 0.5  # rad/s

    #Timer Callback
    def timer_cb(self):
        elapsed_time = (self.get_clock().now() - self.start_time).nanoseconds / 1e9
        angle = elapsed_time * self.angular_speed
        
        # DINÁMICO: map -> odom ---
        # Simula el movimiento del robot siguiendo una trayectoria circular
        self.odom_transform.header.stamp = self.get_clock().now().to_msg()
        self.odom_transform.header.frame_id = 'map'
        self.odom_transform.child_frame_id = 'odom'
        self.odom_transform.transform.translation.x = 0.5 #0.0
        self.odom_transform.transform.translation.y = 0.0
        self.odom_transform.transform.translation.z = 0.0
        
        q_static = transforms3d.euler.euler2quat(0, 0, 0)
        self.odom_transform.transform.rotation.x = q_static[1]
        self.odom_transform.transform.rotation.y = q_static[2]
        self.odom_transform.transform.rotation.z = q_static[3]
        self.odom_transform.transform.rotation.w = q_static[0]

        # DINÁMICO: odom -> base_footprint
        self.base_footprint_transform.header.stamp = self.get_clock().now().to_msg()
        self.base_footprint_transform.header.frame_id = 'odom'
        self.base_footprint_transform.child_frame_id = 'base_footprint'

        # Ecuaciones paramétricas de un círculo: x = r*cos(th), y = r*sin(th)
        self.base_footprint_transform.transform.translation.x = self.radius * np.cos(angle)
        self.base_footprint_transform.transform.translation.y = self.radius * np.sin(angle)
        self.base_footprint_transform.transform.translation.z = 0.0

        # El robot debe ver hacia el frente de su trayectoria (tangente al círculo)
        q_yaw = transforms3d.euler.euler2quat(0, 0, angle + np.pi/2)
        self.base_footprint_transform.transform.rotation.x = q_yaw[1]
        self.base_footprint_transform.transform.rotation.y = q_yaw[2]
        self.base_footprint_transform.transform.rotation.z = q_yaw[3]
        self.base_footprint_transform.transform.rotation.w = q_yaw[0]

        # DINÁMICO: Grito de Ruedas (base_link -> wheel_r/l)
        # Simula que las ruedas están girando mientras el robot se mueve
        # Se usa 'angle * 3' para que las ruedas giren visualmente más rápido

        #Rueda derecha
        self.wheel_r_transform.header.stamp = self.get_clock().now().to_msg()
        self.wheel_r_transform.header.frame_id = 'base_link'
        self.wheel_r_transform.child_frame_id = 'wheel_r'
        self.wheel_r_transform.transform.translation.x = 0.052
        self.wheel_r_transform.transform.translation.y = -0.095
        self.wheel_r_transform.transform.translation.z = -0.0025
        q_wheel = transforms3d.euler.euler2quat(0, angle * 3, 0) # Rotación sobre el eje Y
        self.wheel_r_transform.transform.rotation.x = q_wheel[1]
        self.wheel_r_transform.transform.rotation.y = q_wheel[2]
        self.wheel_r_transform.transform.rotation.z = q_wheel[3]
        self.wheel_r_transform.transform.rotation.w = q_wheel[0]

        #Rueda izquierda
        self.wheel_l_transform.header.stamp = self.get_clock().now().to_msg()
        self.wheel_l_transform.header.frame_id = 'base_link'
        self.wheel_l_transform.child_frame_id = 'wheel_l'
        self.wheel_l_transform.transform.translation.x = 0.052
        self.wheel_l_transform.transform.translation.y = 0.095
        self.wheel_l_transform.transform.translation.z = -0.0025
        q_wheel = transforms3d.euler.euler2quat(0, angle * 3, 0) # Rotación sobre el eje Y
        self.wheel_l_transform.transform.rotation.x = q_wheel[1]
        self.wheel_l_transform.transform.rotation.y = q_wheel[2]
        self.wheel_l_transform.transform.rotation.z = q_wheel[3]
        self.wheel_l_transform.transform.rotation.w = q_wheel[0]

        # Publicar todas las transformaciones calculadas en este ciclo
        self.tf_odom.sendTransform(self.odom_transform)
        self.tf_base_footprint.sendTransform(self.base_footprint_transform)
        self.tf_wheel_r.sendTransform(self.wheel_r_transform)
        self.tf_wheel_l.sendTransform(self.wheel_l_transform)


def main(args=None):
    rclpy.init(args=args)
    node = PuzzlebotPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            rclpy.shutdown()
        node.destroy_node()

if __name__ == '__main__':
    main()