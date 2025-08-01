import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
import tkinter as tk


class TargetGUI(Node):
    def __init__(self):
        super().__init__('target_gui')
        self.publisher_ = self.create_publisher(PoseStamped, '/target_pose', 10)

        # GUI arayüzü
        self.root = tk.Tk()
        self.root.title("🎯 Set Target Position (Y-Z Plane)")

        # Giriş etiketleri
        tk.Label(self.root, text="Y position (sideways):").grid(row=0, column=0, padx=10, pady=5, sticky='e')
        tk.Label(self.root, text="Z position (height):").grid(row=1, column=0, padx=10, pady=5, sticky='e')

        # Giriş kutuları
        self.y_entry = tk.Entry(self.root, width=10)
        self.z_entry = tk.Entry(self.root, width=10)
        self.y_entry.grid(row=0, column=1, padx=10, pady=5)
        self.z_entry.grid(row=1, column=1, padx=10, pady=5)

        # Varsayılan başlangıç değerleri
        self.y_entry.insert(0, "0.5")
        self.z_entry.insert(0, "0.5")

        # Buton
        send_button = tk.Button(self.root, text="Send Target", command=self.send_target)
        send_button.grid(row=2, column=0, columnspan=2, pady=10)

        # Bilgi etiketi
        self.status_label = tk.Label(self.root, text="", fg="green")
        self.status_label.grid(row=3, column=0, columnspan=2)

    def send_target(self):
        try:
            y = float(self.y_entry.get())
            z = float(self.z_entry.get())
        except ValueError:
            self.status_label.config(text="❗ Invalid input. Please enter numbers.", fg="red")
            self.get_logger().error("Invalid input! Please enter numeric Y and Z.")
            return

        msg = PoseStamped()
        msg.header.frame_id = "end_effector"
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose.position.x = 0.0          # X sabit
        msg.pose.position.y = y
        msg.pose.position.z = z
        msg.pose.orientation.w = 1.0       # Default quaternion (no rotation)

        self.publisher_.publish(msg)
        self.status_label.config(text=f"✅ Sent: y={y:.2f}, z={z:.2f}", fg="green")
        self.get_logger().info(f"✅ Sent target: y={y:.2f}, z={z:.2f}")

    def run(self):
        self.root.mainloop()


def main(args=None):
    rclpy.init(args=args)
    node = TargetGUI()
    try:
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
