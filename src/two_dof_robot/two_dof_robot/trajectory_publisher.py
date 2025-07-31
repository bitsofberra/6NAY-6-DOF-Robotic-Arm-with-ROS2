import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Header


class TrajectoryPublisher(Node):
    def __init__(self):
        super().__init__('trajectory_publisher')

        # Yayıncı ve zamanlayıcı
        self.publisher_ = self.create_publisher(JointState, '/joint_states', 10)
        self.dt = 0.05  # 20 Hz yayın
        self.T = 15.0   # Toplam süre (saniye)
        self.timer = self.create_timer(self.dt, self.publish_trajectory)

        # Başlangıç ve hedef pozisyonlar
        self.q0 = [0.0, 0.0]
        self.qf = [5.0, -0.8]
        self.start_time = self.get_clock().now()

        # Yayın durumu
        self.stop = False

    def s_curve(self, t, T, q0, qf):
        tau = t / T
        tau = min(max(tau, 0.0), 1.0)
        s = 10 * tau**3 - 15 * tau**4 + 6 * tau**5
        return q0 + (qf - q0) * s

    def publish_trajectory(self):
        if self.stop:
            return

        t = (self.get_clock().now() - self.start_time).nanoseconds / 1e9

        if t > self.T:
            self.get_logger().info("Target reached, stopping trajectory publishing.")
            self.stop = True
            return

        positions = [
            self.s_curve(t, self.T, self.q0[0], self.qf[0]),
            self.s_curve(t, self.T, self.q0[1], self.qf[1])
        ]

        msg = JointState()
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = ['joint1', 'joint2']
        msg.position = positions

        self.publisher_.publish(msg)
        self.get_logger().info(f"t={t:.2f}s, positions={positions}")


def main(args=None):
    rclpy.init(args=args)
    node = TrajectoryPublisher()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
