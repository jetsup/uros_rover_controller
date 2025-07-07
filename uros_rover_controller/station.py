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

        self.uros_received_mag_data: Vector3 = Vector3()
        self.uros_received_gyro_data: Vector3 = Vector3()
        self.uros_received_accel_data: Vector3 = Vector3()
        self.uros_received_orientation_data: Vector3 = Vector3()
        self.uros_received_proximity_data: Vector3 = Vector3()

        self.mag_subscription = self.create_subscription(
            Vector3, "uros_mag_data", self.mag_listener_callback, qos_profile
        )
        self.gyro_subscription = self.create_subscription(
            Vector3, "uros_gyro_data", self.gyro_listener_callback, qos_profile
        )
        self.accel_subscription = self.create_subscription(
            Vector3, "uros_accel_data", self.accel_listener_callback, qos_profile
        )
        self.orientation_subscription = self.create_subscription(
            Vector3,
            "uros_orientation_data",
            self.orientation_listener_callback,
            qos_profile,
        )
        self.proximity_subscription = self.create_subscription(
            Vector3,
            "uros_proximity_data",
            self.proximity_listener_callback,
            qos_profile,
        )

        self.debug_print_timer = self.create_timer(0.5, self.debug_print_callback)

    def mag_listener_callback(self, msg):
        self.uros_received_mag_data = msg
        # self.get_logger().info(f"Received Magnetometer Data: {msg}")

    def gyro_listener_callback(self, msg):
        self.uros_received_gyro_data = msg
        # self.get_logger().info(f"Received Gyroscope Data: {msg}")

    def accel_listener_callback(self, msg):
        self.uros_received_accel_data = msg
        # self.get_logger().info(f"Received Accelerometer Data: {msg}")

    def orientation_listener_callback(self, msg):
        self.uros_received_orientation_data = msg
        # self.get_logger().info(f"Received Orientation Data: {msg}")

    def proximity_listener_callback(self, msg):
        self.uros_received_proximity_data = msg
        # self.get_logger().info(f"Received Proximity Data: {msg}")

    def debug_print_callback(self):
        self.get_logger().info(
            f"M: x={self.uros_received_mag_data.x:.4f}, "
            f"y={self.uros_received_mag_data.y:.4f}, "
            f"z={self.uros_received_mag_data.z:.4f} "
            f"G: x={self.uros_received_gyro_data.x:.4f}, "
            f"y={self.uros_received_gyro_data.y:.4f}, "
            f"z={self.uros_received_gyro_data.z:.4f} "
            f"A: x={self.uros_received_accel_data.x:.4f}, "
            f"y={self.uros_received_accel_data.y:.4f}, "
            f"z={self.uros_received_accel_data.z:.4f} "
            f"O: x={self.uros_received_orientation_data.x:.4f}, "
            f"y={self.uros_received_orientation_data.y:.4f}, "
            f"z={self.uros_received_orientation_data.z:.4f} "
            f"P: x={self.uros_received_proximity_data.x:.4f}, "
            f"y={self.uros_received_proximity_data.y:.4f}, "
            f"z={self.uros_received_proximity_data.z:.4f}\n"
        )


def main(args=None):
    rclpy.init(args=args)
    rover_data = RoverData()
    try:
        rclpy.spin(rover_data)
    except KeyboardInterrupt:
        print("Shutting down Rover Data Node.")
    finally:
        try:
            rover_data.destroy_node()
            rclpy.shutdown()
        except Exception as e:
            print(f"Error during shutdown: {e}")
