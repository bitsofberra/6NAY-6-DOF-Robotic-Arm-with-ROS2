# import rclpy
# from rclpy.node import Node
# from geometry_msgs.msg import PoseStamped
# from visualization_msgs.msg import InteractiveMarker, InteractiveMarkerControl, Marker
# from interactive_markers.interactive_marker_server import InteractiveMarkerServer

# class TargetSetter(Node):
#     def __init__(self):
#         super().__init__('interactive_marker_target')

#         self.publisher_ = self.create_publisher(PoseStamped, '/target_pose', 10)
#         self.server = InteractiveMarkerServer(self, "target_marker")

#         # Interactive Marker
#         int_marker = InteractiveMarker()
#         int_marker.header.frame_id = "base"
#         int_marker.name = "target"
#         int_marker.description = "Move me in Z-Y plane"
#         int_marker.scale = 0.3
#         int_marker.pose.position.y = 0.5
#         int_marker.pose.position.z = 0.5

#         # Z-Y düzleminde sürüklenebilir kontrol
#         control = InteractiveMarkerControl()
#         control.name = "move_zy"
#         control.interaction_mode = InteractiveMarkerControl.MOVE_PLANE
#         control.orientation.w = 1.0
#         control.orientation.x = 1.0  # X'e dik → Z-Y düzlemi
#         control.orientation.y = 0.0
#         control.orientation.z = 0.0
#         control.always_visible = True

#         # Görsel Marker
#         marker = Marker()
#         marker.type = Marker.SPHERE
#         marker.scale.x = 0.05
#         marker.scale.y = 0.05
#         marker.scale.z = 0.05
#         marker.color.r = 1.0
#         marker.color.g = 0.0
#         marker.color.b = 0.0
#         marker.color.a = 1.0
#         control.markers.append(marker)

#         int_marker.controls.append(control)

#         # Doğru kullanım — ayrı ayrı çağır
#         self.server.insert(int_marker)
#         self.server.setCallback(int_marker.name, self.process_feedback)
#         self.server.applyChanges()

#     def process_feedback(self, feedback):
#         msg = PoseStamped()
#         msg.header = feedback.header
#         msg.pose = feedback.pose
#         self.publisher_.publish(msg)

#         y = msg.pose.position.y
#         z = msg.pose.position.z
#         self.get_logger().info(f"📍 New Z-Y target: y={y:.2f}, z={z:.2f}")

# def main(args=None):
#     rclpy.init(args=args)
#     node = TargetSetter()
#     try:
#         rclpy.spin(node)
#     except KeyboardInterrupt:
#         pass
#     node.destroy_node()
#     rclpy.shutdown()

# if __name__ == '__main__':
#     main()
