import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Header
import math


class TargetFollower(Node):
    def __init__(self):
        super().__init__('target_follower')
        
        # Joint state publisher (for RViz and robot_state_publisher)
        self.publisher_ = self.create_publisher(JointState, '/joint_states', 10)

        # Target pose subscriber
        self.subscription = self.create_subscription(PoseStamped, '/target_pose', self.pose_callback, 10)

        # Timer for trajectory update
        self.timer = self.create_timer(0.05, self.update_trajectory)

        # Link lengths (Z-Y düzleminde)
        self.l1 = 1.0  # Link1 uzunluğu (dikey)
        self.l2 = 1.0  # Link2 uzunluğu (yatay)

        # Açı bilgileri
        self.current_q = [0.0, 0.0]  # Mevcut açı
        self.target_q = None        # Hedef açı
        self.q0 = [0.0, 0.0]         # Başlangıç açı
        self.qf = [0.0, 0.0]         # Bitiş açı

        # Zamanlama
        self.t = 0.0
        self.T = 5.0  # Hareket süresi (saniye)

    def pose_callback(self, msg):
        # Z-Y düzleminde hedef pozisyon al
        y = msg.pose.position.y
        z = msg.pose.position.z
        try:
            q1, q2 = self.inverse_kinematics(y, z)
            self.q0 = self.current_q
            self.qf = [q1, q2]
            self.target_q = self.qf
            self.t = 0.0

            self.get_logger().info(f"🎯 New target received: (y={y:.2f}, z={z:.2f}) → q=({q1:.2f}, {q2:.2f})")
        except ValueError:
            self.get_logger().warn("❌ Target out of reach.")

    def inverse_kinematics(self, y, z):
        """
        2-DOF planar kol için ters kinematik (Z-Y düzlemi)
        """
        l1, l2 = self.l1, self.l2
        d = math.hypot(y, z)  # Hedefe olan mesafe
        cos_q2 = (d**2 - l1**2 - l2**2) / (2 * l1 * l2)

        if abs(cos_q2) > 1.0:
            raise ValueError("Unreachable target")

        q2 = math.acos(cos_q2)
        k1 = l1 + l2 * math.cos(q2)
        k2 = l2 * math.sin(q2)
        q1 = math.atan2(z, y) - math.atan2(k2, k1)

        return q1, q2

    def s_curve(self, t, T, q0, qf):
        """
        S-curve (5. derece polinom) interpolasyonu
        """
        tau = t / T
        tau = min(max(tau, 0.0), 1.0)
        s = 10 * tau**3 - 15 * tau**4 + 6 * tau**5
        return q0 + (qf - q0) * s

    def update_trajectory(self):
        if self.target_q is None:
            return

        self.t += 0.05
        if self.t > self.T:
            self.current_q = self.qf
            return

        q1 = self.s_curve(self.t, self.T, self.q0[0], self.qf[0])
        q2 = self.s_curve(self.t, self.T, self.q0[1], self.qf[1])

        msg = JointState()
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = ['joint1', 'joint2']
        msg.position = [q1, q2]
        self.publisher_.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = TargetFollower()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
