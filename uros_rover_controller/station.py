import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from geometry_msgs.msg import Vector3


class RoverData(Node):  # subscriber node
    def __init__(self):
        super().__init__("rover_data")

        # Define a QoS profile for best effort communication
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )

        self.mag_subscription = self.create_subscription(
            Vector3, "uros_mag_data", self.listener_callback, qos_profile
        )
        self.gyro_subscription = self.create_subscription(
            Vector3, "uros_gyro_data", self.listener_callback, qos_profile
        )
        self.accel_subscription = self.create_subscription(
            Vector3, "uros_accel_data", self.listener_callback, qos_profile
        )
        self.proximity_subscription = self.create_subscription(
            Vector3, "uros_proximity_data", self.listener_callback, qos_profile
        )

    def listener_callback(self, msg):
        self.get_logger().info(f"Received: {msg}")


def main(args=None):
    rclpy.init(args=args)
    rover_data = RoverData()
    try:
        rclpy.spin(rover_data)
    except KeyboardInterrupt:
        pass
    finally:
        rover_data.destroy_node()
        rclpy.shutdown()
