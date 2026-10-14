import math

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64


class GlobalVelocityNode(Node):
    def __init__(self):
        super().__init__('global_velocity_node')


        self.declare_parameter('wheel_radius', 0.03)       
        self.declare_parameter('wheel_separation', 0.17)   
        self.declare_parameter('initial_theta', 0.0)
        self.declare_parameter('rate_hz', 10.0)            

        self.r = self.get_parameter('wheel_radius').value
        self.s = self.get_parameter('wheel_separation').value
        self.theta = self.get_parameter('initial_theta').value
        rate = self.get_parameter('rate_hz').value

    
        self.phi_dot_L = 0.0
        self.phi_dot_R = 0.0
        self.x = 0.0
        self.y = 0.0
        self.dt = 1.0 / rate


        self.create_subscription(Float64, '/left_wheel/command', self.left_cb, 10)
        self.create_subscription(Float64, '/right_wheel/command', self.right_cb, 10)


        self.create_timer(self.dt, self.update)

        self.get_logger().info(
            f'Node siap. r={self.r} m, s={self.s} m. '
            'Menunggu /left_wheel/command dan /right_wheel/command')

    def left_cb(self, msg: Float64):
        self.phi_dot_L = msg.data

    def right_cb(self, msg: Float64):
        self.phi_dot_R = msg.data

    def update(self):
        # 1) Kecepatan linear tiap roda: V = r * phi_dot
        V_L = self.r * self.phi_dot_L
        V_R = self.r * self.phi_dot_R

        # 2) Kecepatan body frame
        V_B = (V_L + V_R) / 2.0
        omega = (V_R - V_L) / self.s

        # 3) Transformasi ke global frame
        x_dot = V_B * math.cos(self.theta)
        y_dot = V_B * math.sin(self.theta)
        theta_dot = omega

        # 4) Radius lintasan 
        if abs(V_R - V_L) > 1e-9:
            l = self.s * (V_L + V_R) / (2.0 * (V_R - V_L))
        else:
            l = float('inf')

        # 5) Integrasi pose
        self.x += x_dot * self.dt
        self.y += y_dot * self.dt
        self.theta = math.atan2(math.sin(self.theta + theta_dot * self.dt),
                                math.cos(self.theta + theta_dot * self.dt))

        # 6) Tampilan ke terminal 
        self.get_logger().info(
            f'\n  V_L={V_L:.3f} m/s  V_R={V_R:.3f} m/s'
            f'\n  V_B={V_B:.3f} m/s  omega={omega:.3f} rad/s  l={l:.3f} m'
            f'\n  [Global] x_dot={x_dot:.3f} m/s  y_dot={y_dot:.3f} m/s  theta_dot={theta_dot:.3f} rad/s'
            f'\n  [Pose ] x={self.x:.3f} m  y={self.y:.3f} m  theta={math.degrees(self.theta):.1f} deg',
            throttle_duration_sec=0.5)


def main(args=None):
    rclpy.init(args=args)
    node = GlobalVelocityNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()