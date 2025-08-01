import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Header
import math
import numpy as np


class TrajectoryToTarget(Node):
    def __init__(self):
        super().__init__('trajectory_to_target')
        self.publisher_ = self.create_publisher(JointState, '/joint_states', 10)

        # Yayınlama süresi ve zaman kontrolü
        self.dt = 0.05  # 50ms
        self.T = 5.0    # Hedefe varış süresi (saniye)
        self.elapsed_time = 0.0
        self.timer = self.create_timer(self.dt, self.publish_trajectory)

        # Kol uzunlukları (URDF'te belirttiğinle aynı olmalı)
        self.l1 = 1.0
        self.l2 = 1.0

        # Uç efektör konumları
        home_x = 0.0
        home_y = 0.0

        target_x = 1.0
        target_y = 1.0

        # Ters kinematikten başlangıç ve hedef açılarını bul
        self.home_q = self.inverse_kinematics(home_x, home_y)
        self.target_q = self.inverse_kinematics(target_x, target_y)

        self.get_logger().info(f"Home ({home_x}, {home_y}) → {self.home_q}")
        self.get_logger().info(f"Target ({target_x}, {target_y}) → {self.target_q}")

        self.q0 = self.home_q
        self.qf = self.target_q

    def s_curve(self, t, T, q0, qf):
        """S-eğrisi interpolasyonu"""
        tau = t / T
        tau = min(max(tau, 0.0), 1.0)
        s = 10 * tau**3 - 15 * tau**4 + 6 * tau**5
        return q0 + (qf - q0) * s

    def forward_kinematics(self, q):
        """Açılardan uç efektör pozisyonunu hesapla"""
        x = self.l1 * math.cos(q[0]) + self.l2 * math.cos(q[0] + q[1])
        y = self.l1 * math.sin(q[0]) + self.l2 * math.sin(q[0] + q[1])
        z = 0.1  # sabit yükseklik (örnek olarak)
        return [x, y, z]

    def inverse_kinematics(self, x, y):
        """Verilen (x, y) pozisyonu için açıları hesapla"""
        l1 = self.l1
        l2 = self.l2

        cos_q2 = (x**2 + y**2 - l1**2 - l2**2) / (2 * l1 * l2)
        q2 = math.acos(np.clip(cos_q2, -1.0, 1.0))

        k1 = l1 + l2 * math.cos(q2)
        k2 = l2 * math.sin(q2)
        q1 = math.atan2(y, x) - math.atan2(k2, k1)

        return [q1, q2]

    def publish_trajectory(self):
        """Her yayın döngüsünde bir adım ilerleme"""
        self.elapsed_time += self.dt

        if self.elapsed_time > self.T:
            # Yön değiştir
            self.q0, self.qf = self.qf, self.q0
            self.elapsed_time = 0.0
            self.get_logger().info("Reached target. Reversing...")

        # Her iki eklem için interpolasyon
        q = [
            self.s_curve(self.elapsed_time, self.T, self.q0[0], self.qf[0]),
            self.s_curve(self.elapsed_time, self.T, self.q0[1], self.qf[1])
        ]

        # ROS mesajı
        msg = JointState()
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = ['joint1', 'joint2']
        msg.position = q
        self.publisher_.publish(msg)

        # Uç efektör pozisyonunu yazdır
        ee = self.forward_kinematics(q)
        self.get_logger().info(f"t={self.elapsed_time:.2f}s, joints={q}, ee={ee}")


def main(args=None):
    rclpy.init(args=args)
    node = TrajectoryToTarget()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
