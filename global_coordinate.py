
import math

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState


class GlobalCoordinate(Node):
    def __init__(self):
        super().__init__('global_coordinate')

        # Parameter robot
        self.r = 0.03
        self.s = 0.17

        # Nama joint roda
        self.left_joint = 'base_left_wheel_joint'
        self.right_joint = 'base_right_wheel_joint'

        # Posisi dan orientasi robot dalam koordinat global
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0

        # Data sebelumnya
        self.prev_time = None
        self.prev_left_pos = None
        self.prev_right_pos = None

        self.subscription = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_callback,
            10
        )

        self.get_logger().info(
            'Global Coordinate aktif, menunggu data /joint_states...'
        )

    def joint_callback(self, msg):
        # Cari indeks joint roda
        try:            
            left_idx = msg.name.index(self.left_joint)
            right_idx = msg.name.index(self.right_joint)
        except ValueError:
            self.get_logger().warning(
                'Nama joint roda tidak ditemukan di /joint_states.'
            )
            return

        # Pastikan posisi tersedia
        if len(msg.position) <= max(left_idx, right_idx):
            return

        # Waktu simulasi
        current_time = (
            msg.header.stamp.sec
            + msg.header.stamp.nanosec * 1e-9
        )

        left_pos = msg.position[left_idx]
        right_pos = msg.position[right_idx]

        # Sampel pertama hanya untuk inisialisasi
        if self.prev_time is None:
            self.prev_time = current_time
            self.prev_left_pos = left_pos
            self.prev_right_pos = right_pos
            return

        dt = current_time - self.prev_time

        # Abaikan timestamp yang tidak bertambah
        if dt <= 0:
            return

        # Kecepatan sudut roda (rad/s)
        if len(msg.velocity) > max(left_idx, right_idx):
            phi_left = msg.velocity[left_idx]
            phi_right = msg.velocity[right_idx]
        else:
            # Alternatif jika velocity tidak tersedia
            phi_left = (left_pos - self.prev_left_pos) / dt
            phi_right = (right_pos - self.prev_right_pos) / dt

        # Simpan data terbaru
        self.prev_time = current_time
        self.prev_left_pos = left_pos
        self.prev_right_pos = right_pos

        # Kecepatan linear roda (m/s)
        v_left = self.r * phi_left
        v_right = self.r * phi_right

        # Forward kinematics
        v_body = (v_left + v_right) / 2.0
        omega = (v_right - v_left) / self.s

        # Gunakan orientasi tengah selama interval waktu
        theta_mid = self.theta + (omega * dt / 2.0)

        # Kecepatan dalam koordinat global
        vx_global = v_body * math.cos(theta_mid)
        vy_global = v_body * math.sin(theta_mid)

        # Integrasi posisi global
        self.x += vx_global * dt
        self.y += vy_global * dt

        # Integrasi orientasi
        self.theta += omega * dt

        # Batasi theta ke rentang -pi sampai pi
        self.theta = math.atan2(
            math.sin(self.theta),
            math.cos(self.theta)
        )

        # Output terminal seperti contoh temanmu
        self.get_logger().info(
            f'x_dot = {vx_global:.3f} m/s | '
            f'y_dot = {vy_global:.3f} m/s | '
            f'theta_dot = {omega:.3f} rad/s | '
            f'theta = {self.theta:.3f} rad'
        )


def main(args=None):
    rclpy.init(args=args)
    node = GlobalCoordinate()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
