import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Header
import math

class TargetFollower(Node):
    def __init__(self):
        super().__init__('target_follower')
        self.publisher_ = self.create_publisher(JointState, '/joint_states', 10)
        self.subscription = self.create_subscription(PoseStamped, '/target_pose', self.pose_callback, 10)
        self.timer = self.create_timer(0.05, self.update_trajectory)

        # Link lengths (Z-Y düzleminde çözüm yapılır)
        self.l1 = 1.0
        self.l2 = 1.0
        self.reach = self.l1 + self.l2

        self.current_q = [0.0, 0.0]
        self.target_q = None
        self.q0 = [0.0, 0.0]
        self.qf = [0.0, 0.0]

        self.t = 0.0
        self.T = 3.0

    def pose_callback(self, msg):
        x = msg.pose.position.x  # şu an x = 0.0 olacak hep
        y = msg.pose.position.y
        z = msg.pose.position.z

        # Marker zaten spherical shell'e projekte ediliyor
        r = math.sqrt(x**2 + y**2 + z**2)
        if abs(r - self.reach) > 0.01:
            self.get_logger().warn("❌ Marker is off shell.")
            return

        try:
            q1, q2 = self.inverse_kinematics(y, z)
            self.q0 = self.current_q
            self.qf = [q1, q2]
            self.target_q = self.qf
            self.t = 0.0
            self.get_logger().info(f"🎯 Target (y={y:.2f}, z={z:.2f}) → q1={math.degrees(q1):.1f}°, q2={math.degrees(q2):.1f}°")
        except ValueError:
            self.get_logger().warn("❌ Inverse kinematics failed.")

    def inverse_kinematics(self, y, z):
        d = math.hypot(y, z)
        l1, l2 = self.l1, self.l2

        cos_q2 = (d**2 - l1**2 - l2**2) / (2 * l1 * l2)
        if abs(cos_q2) > 1.0:
            raise ValueError("Out of reach")

        q2 = math.acos(cos_q2)
        k1 = l1 + l2 * math.cos(q2)
        k2 = l2 * math.sin(q2)
        q1 = math.atan2(z, y) - math.atan2(k2, k1)
        return q1, q2

    def s_curve(self, t, T, q0, qf):
        tau = min(max(t / T, 0.0), 1.0)
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
