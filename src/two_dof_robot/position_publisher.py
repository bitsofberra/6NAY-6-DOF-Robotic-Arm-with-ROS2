import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray

class PositionPublisher(Node):
    def __init__(self):
        super().__init__("two_dof_position_publisher")
        self.declare_parameter("publish_topic", "/position_commands")
        self.declare_parameter("wait_sec_between_publish", 3)
        self.declare_parameter("goal_names", ["home", "goal"])

        wait_time = self.get_parameter("wait_sec_between_publish").value
        topic = self.get_parameter("publish_topic").value
        goal_names = self.get_parameter("goal_names").value

        self.goals = []
        for name in goal_names:
            self.declare_parameter(name, rclpy.Parameter.Type.DOUBLE_ARRAY)
            self.goals.append(self.get_parameter(name).value)

        self.publisher_ = self.create_publisher(Float64MultiArray, topic, 10)
        self.timer = self.create_timer(wait_time, self.timer_callback)
        self.index = 0

    def timer_callback(self):
        msg = Float64MultiArray()
        msg.data = self.goals[self.index]
        self.get_logger().info(f"Publishing: {msg.data}")
        self.publisher_.publish(msg)
        self.index = (self.index + 1) % len(self.goals)

def main(args=None):
    rclpy.init(args=args)
    node = PositionPublisher()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()
